; Auto-generated. Do not edit!


(cl:in-package robot_serving-msg)


;//! \htmlinclude PMPTraj.msg.html

(cl:defclass <PMPTraj> (roslisp-msg-protocol:ros-message)
  ((traj
    :reader traj
    :initarg :traj
    :type (cl:vector robot_serving-msg:PMPPoint)
   :initform (cl:make-array 0 :element-type 'robot_serving-msg:PMPPoint :initial-element (cl:make-instance 'robot_serving-msg:PMPPoint)))
   (time_step
    :reader time_step
    :initarg :time_step
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PMPTraj (<PMPTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PMPTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PMPTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-msg:<PMPTraj> is deprecated: use robot_serving-msg:PMPTraj instead.")))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <PMPTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:traj-val is deprecated.  Use robot_serving-msg:traj instead.")
  (traj m))

(cl:ensure-generic-function 'time_step-val :lambda-list '(m))
(cl:defmethod time_step-val ((m <PMPTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:time_step-val is deprecated.  Use robot_serving-msg:time_step instead.")
  (time_step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PMPTraj>) ostream)
  "Serializes a message object of type '<PMPTraj>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traj))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'time_step))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'time_step))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PMPTraj>) istream)
  "Deserializes a message object of type '<PMPTraj>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traj) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traj)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robot_serving-msg:PMPPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'time_step) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'time_step)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PMPTraj>)))
  "Returns string type for a message object of type '<PMPTraj>"
  "robot_serving/PMPTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PMPTraj)))
  "Returns string type for a message object of type 'PMPTraj"
  "robot_serving/PMPTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PMPTraj>)))
  "Returns md5sum for a message object of type '<PMPTraj>"
  "35272ff336131608380eedb24f1d6824")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PMPTraj)))
  "Returns md5sum for a message object of type 'PMPTraj"
  "35272ff336131608380eedb24f1d6824")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PMPTraj>)))
  "Returns full string definition for message of type '<PMPTraj>"
  (cl:format cl:nil "# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PMPTraj)))
  "Returns full string definition for message of type 'PMPTraj"
  (cl:format cl:nil "# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PMPTraj>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'time_step) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PMPTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'PMPTraj
    (cl:cons ':traj (traj msg))
    (cl:cons ':time_step (time_step msg))
))
