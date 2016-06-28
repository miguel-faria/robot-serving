;; Auto-generated. Do not edit!


(when (boundp 'robot_serving::PMPPoint)
  (if (not (find-package "ROBOT_SERVING"))
    (make-package "ROBOT_SERVING"))
  (shadow 'PMPPoint (find-package "ROBOT_SERVING")))
(unless (find-package "ROBOT_SERVING::PMPPOINT")
  (make-package "ROBOT_SERVING::PMPPOINT"))

(in-package "ROS")
;;//! \htmlinclude PMPPoint.msg.html


(defclass robot_serving::PMPPoint
  :super ros::object
  :slots (_joint_angles ))

(defmethod robot_serving::PMPPoint
  (:init
   (&key
    ((:joint_angles __joint_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _joint_angles __joint_angles)
   self)
  (:joint_angles
   (&optional __joint_angles)
   (if __joint_angles (setq _joint_angles __joint_angles)) _joint_angles)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_angles
    (* 8    (length _joint_angles)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_angles
     (write-long (length _joint_angles) s)
     (dotimes (i (length _joint_angles))
       (sys::poke (elt _joint_angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get robot_serving::PMPPoint :md5sum-) "9eebf9cc7d525d143ad033b65dacb648")
(setf (get robot_serving::PMPPoint :datatype-) "robot_serving/PMPPoint")
(setf (get robot_serving::PMPPoint :definition-)
      "# Value of joint angles for one DOF
float64[] 	joint_angles 	# sequence of angle values for one joint

")



(provide :robot_serving/PMPPoint "9eebf9cc7d525d143ad033b65dacb648")


