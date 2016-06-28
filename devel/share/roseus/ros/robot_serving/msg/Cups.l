;; Auto-generated. Do not edit!


(when (boundp 'robot_serving::Cups)
  (if (not (find-package "ROBOT_SERVING"))
    (make-package "ROBOT_SERVING"))
  (shadow 'Cups (find-package "ROBOT_SERVING")))
(unless (find-package "ROBOT_SERVING::CUPS")
  (make-package "ROBOT_SERVING::CUPS"))

(in-package "ROS")
;;//! \htmlinclude Cups.msg.html


(defclass robot_serving::Cups
  :super ros::object
  :slots (_n_cups _cups_color _cups_pos_x _cups_pos_y _cups_pos_z _depth_width _depth_height ))

(defmethod robot_serving::Cups
  (:init
   (&key
    ((:n_cups __n_cups) 0)
    ((:cups_color __cups_color) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:cups_pos_x __cups_pos_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cups_pos_y __cups_pos_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cups_pos_z __cups_pos_z) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:depth_width __depth_width) 0)
    ((:depth_height __depth_height) 0)
    )
   (send-super :init)
   (setq _n_cups (round __n_cups))
   (setq _cups_color __cups_color)
   (setq _cups_pos_x __cups_pos_x)
   (setq _cups_pos_y __cups_pos_y)
   (setq _cups_pos_z __cups_pos_z)
   (setq _depth_width (round __depth_width))
   (setq _depth_height (round __depth_height))
   self)
  (:n_cups
   (&optional __n_cups)
   (if __n_cups (setq _n_cups __n_cups)) _n_cups)
  (:cups_color
   (&optional __cups_color)
   (if __cups_color (setq _cups_color __cups_color)) _cups_color)
  (:cups_pos_x
   (&optional __cups_pos_x)
   (if __cups_pos_x (setq _cups_pos_x __cups_pos_x)) _cups_pos_x)
  (:cups_pos_y
   (&optional __cups_pos_y)
   (if __cups_pos_y (setq _cups_pos_y __cups_pos_y)) _cups_pos_y)
  (:cups_pos_z
   (&optional __cups_pos_z)
   (if __cups_pos_z (setq _cups_pos_z __cups_pos_z)) _cups_pos_z)
  (:depth_width
   (&optional __depth_width)
   (if __depth_width (setq _depth_width __depth_width)) _depth_width)
  (:depth_height
   (&optional __depth_height)
   (if __depth_height (setq _depth_height __depth_height)) _depth_height)
  (:serialization-length
   ()
   (+
    ;; uint32 _n_cups
    4
    ;; string[] _cups_color
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _cups_color)) 4
    ;; float32[] _cups_pos_x
    (* 4    (length _cups_pos_x)) 4
    ;; float32[] _cups_pos_y
    (* 4    (length _cups_pos_y)) 4
    ;; float32[] _cups_pos_z
    (* 4    (length _cups_pos_z)) 4
    ;; int32 _depth_width
    4
    ;; int32 _depth_height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _n_cups
       (write-long _n_cups s)
     ;; string[] _cups_color
     (write-long (length _cups_color) s)
     (dolist (elem _cups_color)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _cups_pos_x
     (write-long (length _cups_pos_x) s)
     (dotimes (i (length _cups_pos_x))
       (sys::poke (elt _cups_pos_x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _cups_pos_y
     (write-long (length _cups_pos_y) s)
     (dotimes (i (length _cups_pos_y))
       (sys::poke (elt _cups_pos_y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _cups_pos_z
     (write-long (length _cups_pos_z) s)
     (dotimes (i (length _cups_pos_z))
       (sys::poke (elt _cups_pos_z i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32 _depth_width
       (write-long _depth_width s)
     ;; int32 _depth_height
       (write-long _depth_height s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _n_cups
     (setq _n_cups (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string[] _cups_color
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cups_color (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _cups_color i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _cups_pos_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cups_pos_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cups_pos_x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _cups_pos_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cups_pos_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cups_pos_y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _cups_pos_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cups_pos_z (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cups_pos_z i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32 _depth_width
     (setq _depth_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _depth_height
     (setq _depth_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_serving::Cups :md5sum-) "3a9d0ae4758d926f4f7c5e0d4fe36472")
(setf (get robot_serving::Cups :datatype-) "robot_serving/Cups")
(setf (get robot_serving::Cups :definition-)
      "# This message contains information about the position of n colored cups
uint32 		n_cups 		#number of cups recorded in the workspace
string[] 	cups_color 	#vector with the designations given to each cup
float32[] 	cups_pos_x 	#this is a n matrix with the x position of each cup (in millimeters)
float32[] 	cups_pos_y 	#this is a n matrix with the y position of each cup (in millimeters)
float32[] 	cups_pos_z 	#this is a n matrix with the z position of each cup (in millimeters)
int32 		depth_width 	#this is the width of the depth image (in pixels)
int32 		depth_height 	#this is the height of the depth image (in pixels) 
")



(provide :robot_serving/Cups "3a9d0ae4758d926f4f7c5e0d4fe36472")


