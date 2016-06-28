; Auto-generated. Do not edit!


(cl:in-package robot_serving-srv)


;//! \htmlinclude RobotMovementSendTrajectory-request.msg.html

(cl:defclass <RobotMovementSendTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((limb
    :reader limb
    :initarg :limb
    :type cl:string
    :initform "")
   (robot_trajectory
    :reader robot_trajectory
    :initarg :robot_trajectory
    :type robot_serving-msg:PMPTraj
    :initform (cl:make-instance 'robot_serving-msg:PMPTraj)))
)

(cl:defclass RobotMovementSendTrajectory-request (<RobotMovementSendTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMovementSendTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMovementSendTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<RobotMovementSendTrajectory-request> is deprecated: use robot_serving-srv:RobotMovementSendTrajectory-request instead.")))

(cl:ensure-generic-function 'limb-val :lambda-list '(m))
(cl:defmethod limb-val ((m <RobotMovementSendTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:limb-val is deprecated.  Use robot_serving-srv:limb instead.")
  (limb m))

(cl:ensure-generic-function 'robot_trajectory-val :lambda-list '(m))
(cl:defmethod robot_trajectory-val ((m <RobotMovementSendTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:robot_trajectory-val is deprecated.  Use robot_serving-srv:robot_trajectory instead.")
  (robot_trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMovementSendTrajectory-request>) ostream)
  "Serializes a message object of type '<RobotMovementSendTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'limb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'limb))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMovementSendTrajectory-request>) istream)
  "Deserializes a message object of type '<RobotMovementSendTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'limb) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'limb) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMovementSendTrajectory-request>)))
  "Returns string type for a service object of type '<RobotMovementSendTrajectory-request>"
  "robot_serving/RobotMovementSendTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementSendTrajectory-request)))
  "Returns string type for a service object of type 'RobotMovementSendTrajectory-request"
  "robot_serving/RobotMovementSendTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMovementSendTrajectory-request>)))
  "Returns md5sum for a message object of type '<RobotMovementSendTrajectory-request>"
  "49578c603e30c444612b55184f196db2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMovementSendTrajectory-request)))
  "Returns md5sum for a message object of type 'RobotMovementSendTrajectory-request"
  "49578c603e30c444612b55184f196db2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMovementSendTrajectory-request>)))
  "Returns full string definition for message of type '<RobotMovementSendTrajectory-request>"
  (cl:format cl:nil "~%string 	limb~%PMPTraj robot_trajectory~%~%================================================================================~%MSG: robot_serving/PMPTraj~%# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMovementSendTrajectory-request)))
  "Returns full string definition for message of type 'RobotMovementSendTrajectory-request"
  (cl:format cl:nil "~%string 	limb~%PMPTraj robot_trajectory~%~%================================================================================~%MSG: robot_serving/PMPTraj~%# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMovementSendTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'limb))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMovementSendTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMovementSendTrajectory-request
    (cl:cons ':limb (limb msg))
    (cl:cons ':robot_trajectory (robot_trajectory msg))
))
;//! \htmlinclude RobotMovementSendTrajectory-response.msg.html

(cl:defclass <RobotMovementSendTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((movement_status
    :reader movement_status
    :initarg :movement_status
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotMovementSendTrajectory-response (<RobotMovementSendTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMovementSendTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMovementSendTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<RobotMovementSendTrajectory-response> is deprecated: use robot_serving-srv:RobotMovementSendTrajectory-response instead.")))

(cl:ensure-generic-function 'movement_status-val :lambda-list '(m))
(cl:defmethod movement_status-val ((m <RobotMovementSendTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:movement_status-val is deprecated.  Use robot_serving-srv:movement_status instead.")
  (movement_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMovementSendTrajectory-response>) ostream)
  "Serializes a message object of type '<RobotMovementSendTrajectory-response>"
  (cl:let* ((signed (cl:slot-value msg 'movement_status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMovementSendTrajectory-response>) istream)
  "Deserializes a message object of type '<RobotMovementSendTrajectory-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'movement_status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMovementSendTrajectory-response>)))
  "Returns string type for a service object of type '<RobotMovementSendTrajectory-response>"
  "robot_serving/RobotMovementSendTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementSendTrajectory-response)))
  "Returns string type for a service object of type 'RobotMovementSendTrajectory-response"
  "robot_serving/RobotMovementSendTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMovementSendTrajectory-response>)))
  "Returns md5sum for a message object of type '<RobotMovementSendTrajectory-response>"
  "49578c603e30c444612b55184f196db2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMovementSendTrajectory-response)))
  "Returns md5sum for a message object of type 'RobotMovementSendTrajectory-response"
  "49578c603e30c444612b55184f196db2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMovementSendTrajectory-response>)))
  "Returns full string definition for message of type '<RobotMovementSendTrajectory-response>"
  (cl:format cl:nil "~%int32 	movement_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMovementSendTrajectory-response)))
  "Returns full string definition for message of type 'RobotMovementSendTrajectory-response"
  (cl:format cl:nil "~%int32 	movement_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMovementSendTrajectory-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMovementSendTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMovementSendTrajectory-response
    (cl:cons ':movement_status (movement_status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotMovementSendTrajectory)))
  'RobotMovementSendTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotMovementSendTrajectory)))
  'RobotMovementSendTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementSendTrajectory)))
  "Returns string type for a service object of type '<RobotMovementSendTrajectory>"
  "robot_serving/RobotMovementSendTrajectory")