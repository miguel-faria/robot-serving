;; Auto-generated. Do not edit!


(when (boundp 'robot_serving::Movement)
  (if (not (find-package "ROBOT_SERVING"))
    (make-package "ROBOT_SERVING"))
  (shadow 'Movement (find-package "ROBOT_SERVING")))
(unless (find-package "ROBOT_SERVING::MOVEMENT")
  (make-package "ROBOT_SERVING::MOVEMENT"))
(unless (find-package "ROBOT_SERVING::MOVEMENTREQUEST")
  (make-package "ROBOT_SERVING::MOVEMENTREQUEST"))
(unless (find-package "ROBOT_SERVING::MOVEMENTRESPONSE")
  (make-package "ROBOT_SERVING::MOVEMENTRESPONSE"))

(in-package "ROS")





(defclass robot_serving::MovementRequest
  :super ros::object
  :slots (_x_pos _y_pos _z_pos ))

(defmethod robot_serving::MovementRequest
  (:init
   (&key
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:z_pos __z_pos) 0.0)
    )
   (send-super :init)
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _z_pos (float __z_pos))
   self)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:z_pos
   (&optional __z_pos)
   (if __z_pos (setq _z_pos __z_pos)) _z_pos)
  (:serialization-length
   ()
   (+
    ;; float64 _x_pos
    8
    ;; float64 _y_pos
    8
    ;; float64 _z_pos
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_pos
       (sys::poke _z_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_pos
     (setq _x_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_pos
     (setq _y_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_pos
     (setq _z_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass robot_serving::MovementResponse
  :super ros::object
  :slots (_robot_trajectory ))

(defmethod robot_serving::MovementResponse
  (:init
   (&key
    ((:robot_trajectory __robot_trajectory) (instance robot_serving::PMPTraj :init))
    )
   (send-super :init)
   (setq _robot_trajectory __robot_trajectory)
   self)
  (:robot_trajectory
   (&rest __robot_trajectory)
   (if (keywordp (car __robot_trajectory))
       (send* _robot_trajectory __robot_trajectory)
     (progn
       (if __robot_trajectory (setq _robot_trajectory (car __robot_trajectory)))
       _robot_trajectory)))
  (:serialization-length
   ()
   (+
    ;; robot_serving/PMPTraj _robot_trajectory
    (send _robot_trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_serving/PMPTraj _robot_trajectory
       (send _robot_trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_serving/PMPTraj _robot_trajectory
     (send _robot_trajectory :deserialize buf ptr-) (incf ptr- (send _robot_trajectory :serialization-length))
   ;;
   self)
  )

(defclass robot_serving::Movement
  :super ros::object
  :slots ())

(setf (get robot_serving::Movement :md5sum-) "603e8705159b0ff81a425568466a053a")
(setf (get robot_serving::Movement :datatype-) "robot_serving/Movement")
(setf (get robot_serving::Movement :request) robot_serving::MovementRequest)
(setf (get robot_serving::Movement :response) robot_serving::MovementResponse)

(defmethod robot_serving::MovementRequest
  (:response () (instance robot_serving::MovementResponse :init)))

(setf (get robot_serving::MovementRequest :md5sum-) "603e8705159b0ff81a425568466a053a")
(setf (get robot_serving::MovementRequest :datatype-) "robot_serving/MovementRequest")
(setf (get robot_serving::MovementRequest :definition-)
      "
float64 	x_pos
float64 	y_pos
float64 	z_pos
---

PMPTraj 	robot_trajectory

================================================================================
MSG: robot_serving/PMPTraj
# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length
PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.
float64[] 	time_step	# times of observations, in seconds, starting at zero
================================================================================
MSG: robot_serving/PMPPoint
# Value of joint angles for one DOF
float64[] 	joint_angles 	# sequence of angle values for one joint
")

(setf (get robot_serving::MovementResponse :md5sum-) "603e8705159b0ff81a425568466a053a")
(setf (get robot_serving::MovementResponse :datatype-) "robot_serving/MovementResponse")
(setf (get robot_serving::MovementResponse :definition-)
      "
float64 	x_pos
float64 	y_pos
float64 	z_pos
---

PMPTraj 	robot_trajectory

================================================================================
MSG: robot_serving/PMPTraj
# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length
PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.
float64[] 	time_step	# times of observations, in seconds, starting at zero
================================================================================
MSG: robot_serving/PMPPoint
# Value of joint angles for one DOF
float64[] 	joint_angles 	# sequence of angle values for one joint
")



(provide :robot_serving/Movement "603e8705159b0ff81a425568466a053a")


