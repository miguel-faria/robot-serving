/*
 * Movement_Manager.h
 *
 *  Created on: May 20, 2016
 *      Author: miguel
 */

#ifndef MOVEMENT_DECISION_MOVEMENT_MANAGER_H_
#define MOVEMENT_DECISION_MOVEMENT_MANAGER_H_

#include <opencv2/core/core.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <algorithm>
#include <map>
#include <string>
#include <utility>

#include "../../../devel/include/robot_serving/Cups.h"
#include "../../../devel/include/robot_serving/PMPTraj.h"
#include "../Trajectory_Types/Mixed_Trajectory.h"
#include "../Trajectory_Types/Single_Trajectory.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace robot_serving;

namespace movement_decision {

	class MovementManager {

		private:

			//Possible stages of the movement of the robot towards a cup
			enum Movement_Stage {
				QUEUED,
				STARTED_MOVEMENT,
				MOVEMENT_SUCCESS,
				MOVEMENT_FAILED
			};

			ros::NodeHandle _nh;
			ros::Subscriber _sub_cups_pos;
			ros::Timer _srvc_timer;
			Trajectory_Manager _traj_manager;

			map<string, Point3f> _cups_pos;
			map<string, double> _cups_dist;
			map<string, bool> _cups_observable;
			map<string, int> _cup_mov_phase;

			PMPTraj _robot_movement;
			Point3f _current_objective;
			string _current_cup_id;
			string _baxter_limb;

			/*
			 * update_cup_info - private method used to update the xyz position, distance and visibility status of
			 * a specific cup
			 *
			 * params:
			 * 		- cup_id 	 - string with the identification of the cup
			 * 		- cup_xyz 	 - 3D position of the cup
			 * 		- point_dist - Euclidean distance between the robot and the cup position
			 */
			void update_cup_info(string cup_id, Point3f cup_xyz, double point_dist);

			/*
			 * movement_finished - private method that verifies if the current robot movement towards the target has
			 * finished, either in success or in failure
			 *
			 * return:
			 * 		- true	- if the movement has finished
			 * 		- false	- if the movement is still under way or waiting to be started
			 */
			bool movement_finished();

		public:

			MovementManager() {};

			MovementManager(string cups_sub_topic, string traj_type, string traj_srvc_name, string send_traj_srv,
					string get_feedback_srv, string restart_srv, string baxter_limb, int timer_time) {
				_sub_cups_pos = _nh.subscribe(cups_sub_topic, 1000, &MovementManager::cups_info_receiver, this);
				_traj_manager = Single_Trajectory(traj_type, _nh, traj_srvc_name,
						send_traj_srv, get_feedback_srv, restart_srv),

				_cups_pos = map<string, Point3f>();
				_cups_dist = map<string, double>();
				_cups_observable = map<string, bool>();
				_cup_mov_phase = map<string, int>();
				_srvc_timer = _nh.createTimer(ros::Duration(timer_time), &MovementManager::movement_decision, this);
				_baxter_limb = baxter_limb;
				_current_cup_id = string();
			}

			MovementManager(string cups_sub_topic, string traj_type, string legible_traj_srv,
					string predictable_traj_srv, string send_traj_srv, string get_feedback_srv, string restart_srv,
					string baxter_limb, int timer_time) {

				_sub_cups_pos = _nh.subscribe(cups_sub_topic, 1000, &MovementManager::cups_info_receiver, this);
				_traj_manager = Mixed_Trajectory(traj_type, _nh, legible_traj_srv, predictable_traj_srv, send_traj_srv,
						get_feedback_srv, restart_srv);

				_cups_pos = map<string, Point3f>();
				_cups_dist = map<string, double>();
				_cups_observable = map<string, bool>();
				_cup_mov_phase = map<string, int>();
				_srvc_timer = _nh.createTimer(ros::Duration(timer_time), &MovementManager::movement_decision, this);
				_baxter_limb = baxter_limb;
				_current_cup_id = string();
			}

			virtual ~MovementManager();

			void insert_cup_info(string cup_id, Point3f xyz, double dist) {
				_cups_pos.insert(pair<string, Point3f>(cup_id, xyz));
				_cups_dist.insert(pair<string, double>(cup_id, dist));
				_cups_dist.insert(pair<string, bool>(cup_id, true));
				_cup_mov_phase.insert(pair<string, int>(cup_id, Movement_Stage::QUEUED));
			}

			template<class T, class U> static bool compare_map_elems(pair<T, U> i, pair<T, U> j){
					return i.second < j.second;
			}

			map<string, Point3f>::iterator choose_next_cup(){
				map<string, double> dists = map<string, double>(_cups_dist);
				pair<string, double> min_elem;
				bool selectable = false;
				while(!selectable){
					min_elem = *min_element(dists.begin(), dists.end(),
							&MovementManager::compare_map_elems<string, double>);
					if((_cup_mov_phase.find(min_elem.first)->second == Movement_Stage::QUEUED) &&
							(_cups_observable.find(min_elem.first)->second))
						selectable = true;
					else
						dists.erase(min_elem.first);
				}
				string closer_point_id = min_elem.first;
				return _cups_pos.find(closer_point_id);
			}


			virtual void cups_info_receiver(const robot_serving::CupsConstPtr& msg);
			virtual void movement_decision(const TimerEvent& event);

	};


}  // namespace movement_decision



#endif /* MOVEMENT_MANAGER_H_ */
