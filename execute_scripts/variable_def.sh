#!/usr/bin/env bash

VisionProcessing_Executable="../devel/lib/robot_serving/vision_processing_color_segmentation"
MovementExecutableSimple="../devel/lib/robot_serving/movement_manager_single_trajectory"
MovementExecutableMixed="../devel/lib/robot_serving/movement_manager_mixed_trajectory"
BaxterConnectionExecutable="../src/Movement/baxter_connection.py"
ColorTopic="/kinect2/hd/image_color_rect"
DepthTopic="/kinect2/sd/image_depth_rect"
CupPosTopic="cups_pub"
CalibParams="../data/calib_data.txt"
LegibleTrajSrv="/movement_decision_legible"
PredictableTrajSrv="/movement_decision_predictable"
StartMovSrv="/baxter_start_trajectory"
FeedbackSrv="/baxter_movement_feedback"
RestartMovSrv="/baxter_stop_start_trajectory"
BaxterArm="right"
DecisionPeriod=5
