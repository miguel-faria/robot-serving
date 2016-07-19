/*
 * Movement_Manager.cpp
 *
 *  Created on: May 27, 2016
 *      Author: miguel
 */

#include "Movement_Manager.h"

#include <actionlib_msgs/GoalStatus.h>
#include <opencv2/core/operations.hpp>
#include <ros/service_client.h>
#include <rosconsole/macros_generated.h>
#include <cmath>
#include <vector>

#include "../../../devel/include/robot_serving/Movement.h"
#include "../../../devel/include/robot_serving/RobotMovementCancelTrajectory.h"
#include "../../../devel/include/robot_serving/RobotMovementFeedback.h"
#include "../../../devel/include/robot_serving/RobotMovementSendTrajectory.h"
#include "../Trajectory_Types/Trajectory_Manager.h"

namespace movement_decision{

	MovementManager::~MovementManager(){
		_sub_cups_pos.shutdown();
		_srvc_timer.stop();
		delete &_traj_manager;

		_cups_dist.clear();
		_cups_observable.clear();
		_cups_pos.clear();
		_cup_mov_phase.clear();
	}

	void MovementManager::update_cup_info(string cup_id, Point3f cup_xyz, double point_dist) {
		_cups_pos.at(cup_id) = cup_xyz;
		_cups_dist.at(cup_id) = point_dist;
		_cups_observable.at(cup_id) = true;
	}

	bool MovementManager::movement_finished(){

		return (_cup_mov_phase.at(_current_cup_id) == Movement_Stage::MOVEMENT_SUCCESS ||
				_cup_mov_phase.at(_current_cup_id) == Movement_Stage::MOVEMENT_FAILED);
	}

	void MovementManager::cups_info_receiver(const CupsConstPtr& msg){
		int n_cups = msg->n_cups;
		double point_dist;
		vector<float> x_vec, y_vec, z_vec;
		vector<string> cup_id = msg->cups_color;
		Point3f cup_xyz;
		map<string,Point3f>::iterator pos_it;
		map<string,double>::iterator dist_it;
		map<string,bool>::iterator obs_it;

		x_vec = msg->cups_pos_x;
		y_vec = msg->cups_pos_y;
		z_vec = msg->cups_pos_z;

		if(_cups_pos.empty()){
			for(int i = 0; i < n_cups; i++){
				cup_xyz = Point3f(x_vec[i], y_vec[i], z_vec[i]);
				point_dist = sqrt(pow(cup_xyz.x*1000,2) + pow(cup_xyz.y*1000,2) + pow(cup_xyz.z*1000,2));
				insert_cup_info(cup_id[i], cup_xyz, point_dist);
			}
		} else{
			for(int i = 0; i < n_cups; i++){
				cup_xyz = Point3f(x_vec[i], y_vec[i], z_vec[i]);
				point_dist = sqrt(pow(cup_xyz.x*1000,2) + pow(cup_xyz.y*1000,2) + pow(cup_xyz.z*1000,2));

				if(_cups_pos.count(cup_id[i]) == 0){
					insert_cup_info(cup_id[i], cup_xyz, point_dist);
				} else{
					dist_it = _cups_dist.find(cup_id[i]);

					//If cup's center of mass moves at least 0.5m update with new info
					if(abs(point_dist - dist_it->second) >= 500){
						update_cup_info(cup_id[i], cup_xyz, point_dist);
					}
				}
			}
		}
	}

	void MovementManager::movement_decision(const TimerEvent& event){

		map<string, Point3f>::iterator objective_pos_it;
		int status_response;
		Movement traj_srv;
		RobotMovementSendTrajectory movement_srv;
		RobotMovementFeedback feedback_srv;
		RobotMovementCancelTrajectory restart_srv;
		int movement_result;
		float objective_dist = sqrtf(pow(_current_objective.x, 2) + pow(_current_objective.y, 2)
				+ pow(_current_objective.z, 2));

		if(_cups_pos.size() > 0){

			//If there is no cup selected or the previous movement has finished
			if(_current_cup_id.empty() || movement_finished()){

				//Get next cup to serve
				objective_pos_it = choose_next_cup();
				_current_cup_id = objective_pos_it->first;
				_current_objective = objective_pos_it->second;

				//Obtain robot trajectory
				traj_srv.request.x_pos = _current_objective.x;
				traj_srv.request.y_pos = _current_objective.y;
				traj_srv.request.z_pos = _current_objective.z;
				if(_traj_manager.choose_trajectory_service(_cups_pos, _cups_dist,
						_current_cup_id, _baxter_limb).call(traj_srv)){
					movement_result = traj_srv.response.movement_success;
					ROS_INFO("Response from movement trajectory service received.\n");
				}else {
					ROS_ERROR("Failed to call service to obtain new trajectory.\n");
				}

				if(movement_result == Movement_Result::SUCCESS){
					_cup_mov_phase.at(_current_cup_id) = Movement_Stage::MOVEMENT_SUCCESS;
				}else if(movement_result == Movement_Result::ABORTED || movement_result == Movement_Result::PREEMPTED){
					_cup_mov_phase.at(_current_cup_id) = Movement_Stage::MOVEMENT_FAILED;
				}

			} else{

				//Check movement state
				feedback_srv.request.limb = _baxter_limb;
				if(_traj_manager.get_feedback_traj_serv().call(feedback_srv)){
					status_response = feedback_srv.response.feedback;
					ROS_INFO("Response from the get feedback service about the %s limb received.\n",
							_baxter_limb.c_str());
				}else{
					ROS_ERROR("Failed to call service to get feed back on movement by the %s limb.\n",
							_baxter_limb.c_str());
				}

				if(status_response == actionlib_msgs::GoalStatus::SUCCEEDED){
					_cup_mov_phase.at(_current_cup_id) = Movement_Stage::MOVEMENT_SUCCESS;
					return;
				} else if(status_response == actionlib_msgs::GoalStatus::ABORTED){
					_cup_mov_phase.at(_current_cup_id) = Movement_Stage::MOVEMENT_FAILED;
					return;
				} else if(status_response == actionlib_msgs::GoalStatus::ACTIVE){

					//Verify if there were changes in the cup's position and visibility(TODO)
					if(abs(objective_dist - _cups_dist.at(_current_cup_id)) >= 500){

						//If position changes significantly calculate trajctory to new target
						_current_objective = _cups_pos.at(_current_cup_id);
						traj_srv.request.x_pos = _current_objective.x;
						traj_srv.request.y_pos = _current_objective.y;
						traj_srv.request.z_pos = _current_objective.z;
						if(_traj_manager.choose_trajectory_service(_cups_pos, _cups_dist,
								_current_cup_id, _baxter_limb).call(traj_srv)){
							movement_result = traj_srv.response.movement_success;
							ROS_INFO("Response from movement trajectory service received.\n");
						}else {
							ROS_ERROR("Failed to call service to obtain new trajectory.\n");
						}

						//Send new trajectory to Baxter and stop previous movement
						restart_srv.request.limb = _baxter_limb;
						restart_srv.request.service_code = 2;
						restart_srv.request.robot_trajectory = _robot_movement;
						if(_traj_manager.get_restart_traj_serv().call(restart_srv)){
							_cup_mov_phase.at(_current_cup_id) = Movement_Stage::STARTED_MOVEMENT;
							ROS_INFO("Response from start new movement service received.\n");
						}else{
							_cup_mov_phase.at(_current_cup_id) = Movement_Stage::QUEUED;
							ROS_ERROR("Failed to call service to start new movement.\n");
						}
					}
				}
			}

		}
	}
}


