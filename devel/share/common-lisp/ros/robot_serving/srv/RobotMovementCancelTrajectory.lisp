; Auto-generated. Do not edit!


(cl:in-package robot_serving-srv)


;//! \htmlinclude RobotMovementCancelTrajectory-request.msg.html

(cl:defclass <RobotMovementCancelTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((limb
    :reader limb
    :initarg :limb
    :type cl:string
    :initform "")
   (service_code
    :reader service_code
    :initarg :service_code
    :type cl:integer
    :initform 0)
   (robot_trajectory
    :reader robot_trajectory
    :initarg :robot_trajectory
    :type robot_serving-msg:PMPTraj
    :initform (cl:make-instance 'robot_serving-msg:PMPTraj)))
)

(cl:defclass RobotMovementCancelTrajectory-request (<RobotMovementCancelTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMovementCancelTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMovementCancelTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<RobotMovementCancelTrajectory-request> is deprecated: use robot_serving-srv:RobotMovementCancelTrajectory-request instead.")))

(cl:ensure-generic-function 'limb-val :lambda-list '(m))
(cl:defmethod limb-val ((m <RobotMovementCancelTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:limb-val is deprecated.  Use robot_serving-srv:limb instead.")
  (limb m))

(cl:ensure-generic-function 'service_code-val :lambda-list '(m))
(cl:defmethod service_code-val ((m <RobotMovementCancelTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:service_code-val is deprecated.  Use robot_serving-srv:service_code instead.")
  (service_code m))

(cl:ensure-generic-function 'robot_trajectory-val :lambda-list '(m))
(cl:defmethod robot_trajectory-val ((m <RobotMovementCancelTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:robot_trajectory-val is deprecated.  Use robot_serving-srv:robot_trajectory instead.")
  (robot_trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMovementCancelTrajectory-request>) ostream)
  "Serializes a message object of type '<RobotMovementCancelTrajectory-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'limb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'limb))
  (cl:let* ((signed (cl:slot-value msg 'service_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMovementCancelTrajectory-request>) istream)
  "Deserializes a message object of type '<RobotMovementCancelTrajectory-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'limb) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'limb) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'service_code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMovementCancelTrajectory-request>)))
  "Returns string type for a service object of type '<RobotMovementCancelTrajectory-request>"
  "robot_serving/RobotMovementCancelTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementCancelTrajectory-request)))
  "Returns string type for a service object of type 'RobotMovementCancelTrajectory-request"
  "robot_serving/RobotMovementCancelTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMovementCancelTrajectory-request>)))
  "Returns md5sum for a message object of type '<RobotMovementCancelTrajectory-request>"
  "ad679c59351570bb04b061c3d5ba035b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMovementCancelTrajectory-request)))
  "Returns md5sum for a message object of type 'RobotMovementCancelTrajectory-request"
  "ad679c59351570bb04b061c3d5ba035b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMovementCancelTrajectory-request>)))
  "Returns full string definition for message of type '<RobotMovementCancelTrajectory-request>"
  (cl:format cl:nil "~%string 	limb~%int32 	service_code~%PMPTraj robot_trajectory~%~%================================================================================~%MSG: robot_serving/PMPTraj~%# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMovementCancelTrajectory-request)))
  "Returns full string definition for message of type 'RobotMovementCancelTrajectory-request"
  (cl:format cl:nil "~%string 	limb~%int32 	service_code~%PMPTraj robot_trajectory~%~%================================================================================~%MSG: robot_serving/PMPTraj~%# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMovementCancelTrajectory-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'limb))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMovementCancelTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMovementCancelTrajectory-request
    (cl:cons ':limb (limb msg))
    (cl:cons ':service_code (service_code msg))
    (cl:cons ':robot_trajectory (robot_trajectory msg))
))
;//! \htmlinclude RobotMovementCancelTrajectory-response.msg.html

(cl:defclass <RobotMovementCancelTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((movement_status
    :reader movement_status
    :initarg :movement_status
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotMovementCancelTrajectory-response (<RobotMovementCancelTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMovementCancelTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMovementCancelTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<RobotMovementCancelTrajectory-response> is deprecated: use robot_serving-srv:RobotMovementCancelTrajectory-response instead.")))

(cl:ensure-generic-function 'movement_status-val :lambda-list '(m))
(cl:defmethod movement_status-val ((m <RobotMovementCancelTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:movement_status-val is deprecated.  Use robot_serving-srv:movement_status instead.")
  (movement_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMovementCancelTrajectory-response>) ostream)
  "Serializes a message object of type '<RobotMovementCancelTrajectory-response>"
  (cl:let* ((signed (cl:slot-value msg 'movement_status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMovementCancelTrajectory-response>) istream)
  "Deserializes a message object of type '<RobotMovementCancelTrajectory-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'movement_status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMovementCancelTrajectory-response>)))
  "Returns string type for a service object of type '<RobotMovementCancelTrajectory-response>"
  "robot_serving/RobotMovementCancelTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementCancelTrajectory-response)))
  "Returns string type for a service object of type 'RobotMovementCancelTrajectory-response"
  "robot_serving/RobotMovementCancelTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMovementCancelTrajectory-response>)))
  "Returns md5sum for a message object of type '<RobotMovementCancelTrajectory-response>"
  "ad679c59351570bb04b061c3d5ba035b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMovementCancelTrajectory-response)))
  "Returns md5sum for a message object of type 'RobotMovementCancelTrajectory-response"
  "ad679c59351570bb04b061c3d5ba035b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMovementCancelTrajectory-response>)))
  "Returns full string definition for message of type '<RobotMovementCancelTrajectory-response>"
  (cl:format cl:nil "~%int32 	movement_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMovementCancelTrajectory-response)))
  "Returns full string definition for message of type 'RobotMovementCancelTrajectory-response"
  (cl:format cl:nil "~%int32 	movement_status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMovementCancelTrajectory-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMovementCancelTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMovementCancelTrajectory-response
    (cl:cons ':movement_status (movement_status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotMovementCancelTrajectory)))
  'RobotMovementCancelTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotMovementCancelTrajectory)))
  'RobotMovementCancelTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementCancelTrajectory)))
  "Returns string type for a service object of type '<RobotMovementCancelTrajectory>"
  "robot_serving/RobotMovementCancelTrajectory")