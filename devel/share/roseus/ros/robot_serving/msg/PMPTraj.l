;; Auto-generated. Do not edit!


(when (boundp 'robot_serving::PMPTraj)
  (if (not (find-package "ROBOT_SERVING"))
    (make-package "ROBOT_SERVING"))
  (shadow 'PMPTraj (find-package "ROBOT_SERVING")))
(unless (find-package "ROBOT_SERVING::PMPTRAJ")
  (make-package "ROBOT_SERVING::PMPTRAJ"))

(in-package "ROS")
;;//! \htmlinclude PMPTraj.msg.html


(defclass robot_serving::PMPTraj
  :super ros::object
  :slots (_traj _time_step ))

(defmethod robot_serving::PMPTraj
  (:init
   (&key
    ((:traj __traj) (let (r) (dotimes (i 0) (push (instance robot_serving::PMPPoint :init) r)) r))
    ((:time_step __time_step) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _traj __traj)
   (setq _time_step __time_step)
   self)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:time_step
   (&optional __time_step)
   (if __time_step (setq _time_step __time_step)) _time_step)
  (:serialization-length
   ()
   (+
    ;; robot_serving/PMPPoint[] _traj
    (apply #'+ (send-all _traj :serialization-length)) 4
    ;; float64[] _time_step
    (* 8    (length _time_step)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robot_serving/PMPPoint[] _traj
     (write-long (length _traj) s)
     (dolist (elem _traj)
       (send elem :serialize s)
       )
     ;; float64[] _time_step
     (write-long (length _time_step) s)
     (dotimes (i (length _time_step))
       (sys::poke (elt _time_step i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robot_serving/PMPPoint[] _traj
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj (let (r) (dotimes (i n) (push (instance robot_serving::PMPPoint :init) r)) r))
     (dolist (elem- _traj)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _time_step
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time_step (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time_step i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get robot_serving::PMPTraj :md5sum-) "35272ff336131608380eedb24f1d6824")
(setf (get robot_serving::PMPTraj :datatype-) "robot_serving/PMPTraj")
(setf (get robot_serving::PMPTraj :definition-)
      "# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length
PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.
float64[] 	time_step	# times of observations, in seconds, starting at zero
================================================================================
MSG: robot_serving/PMPPoint
# Value of joint angles for one DOF
float64[] 	joint_angles 	# sequence of angle values for one joint

")



(provide :robot_serving/PMPTraj "35272ff336131608380eedb24f1d6824")


