; Auto-generated. Do not edit!


(cl:in-package robot_serving-srv)


;//! \htmlinclude Movement-request.msg.html

(cl:defclass <Movement-request> (roslisp-msg-protocol:ros-message)
  ((x_pos
    :reader x_pos
    :initarg :x_pos
    :type cl:float
    :initform 0.0)
   (y_pos
    :reader y_pos
    :initarg :y_pos
    :type cl:float
    :initform 0.0)
   (z_pos
    :reader z_pos
    :initarg :z_pos
    :type cl:float
    :initform 0.0))
)

(cl:defclass Movement-request (<Movement-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Movement-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Movement-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<Movement-request> is deprecated: use robot_serving-srv:Movement-request instead.")))

(cl:ensure-generic-function 'x_pos-val :lambda-list '(m))
(cl:defmethod x_pos-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:x_pos-val is deprecated.  Use robot_serving-srv:x_pos instead.")
  (x_pos m))

(cl:ensure-generic-function 'y_pos-val :lambda-list '(m))
(cl:defmethod y_pos-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:y_pos-val is deprecated.  Use robot_serving-srv:y_pos instead.")
  (y_pos m))

(cl:ensure-generic-function 'z_pos-val :lambda-list '(m))
(cl:defmethod z_pos-val ((m <Movement-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:z_pos-val is deprecated.  Use robot_serving-srv:z_pos instead.")
  (z_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Movement-request>) ostream)
  "Serializes a message object of type '<Movement-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Movement-request>) istream)
  "Deserializes a message object of type '<Movement-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_pos) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_pos) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_pos) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Movement-request>)))
  "Returns string type for a service object of type '<Movement-request>"
  "robot_serving/MovementRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Movement-request)))
  "Returns string type for a service object of type 'Movement-request"
  "robot_serving/MovementRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Movement-request>)))
  "Returns md5sum for a message object of type '<Movement-request>"
  "603e8705159b0ff81a425568466a053a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Movement-request)))
  "Returns md5sum for a message object of type 'Movement-request"
  "603e8705159b0ff81a425568466a053a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Movement-request>)))
  "Returns full string definition for message of type '<Movement-request>"
  (cl:format cl:nil "~%float64 	x_pos~%float64 	y_pos~%float64 	z_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Movement-request)))
  "Returns full string definition for message of type 'Movement-request"
  (cl:format cl:nil "~%float64 	x_pos~%float64 	y_pos~%float64 	z_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Movement-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Movement-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Movement-request
    (cl:cons ':x_pos (x_pos msg))
    (cl:cons ':y_pos (y_pos msg))
    (cl:cons ':z_pos (z_pos msg))
))
;//! \htmlinclude Movement-response.msg.html

(cl:defclass <Movement-response> (roslisp-msg-protocol:ros-message)
  ((robot_trajectory
    :reader robot_trajectory
    :initarg :robot_trajectory
    :type robot_serving-msg:PMPTraj
    :initform (cl:make-instance 'robot_serving-msg:PMPTraj)))
)

(cl:defclass Movement-response (<Movement-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Movement-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Movement-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<Movement-response> is deprecated: use robot_serving-srv:Movement-response instead.")))

(cl:ensure-generic-function 'robot_trajectory-val :lambda-list '(m))
(cl:defmethod robot_trajectory-val ((m <Movement-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:robot_trajectory-val is deprecated.  Use robot_serving-srv:robot_trajectory instead.")
  (robot_trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Movement-response>) ostream)
  "Serializes a message object of type '<Movement-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Movement-response>) istream)
  "Deserializes a message object of type '<Movement-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Movement-response>)))
  "Returns string type for a service object of type '<Movement-response>"
  "robot_serving/MovementResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Movement-response)))
  "Returns string type for a service object of type 'Movement-response"
  "robot_serving/MovementResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Movement-response>)))
  "Returns md5sum for a message object of type '<Movement-response>"
  "603e8705159b0ff81a425568466a053a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Movement-response)))
  "Returns md5sum for a message object of type 'Movement-response"
  "603e8705159b0ff81a425568466a053a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Movement-response>)))
  "Returns full string definition for message of type '<Movement-response>"
  (cl:format cl:nil "~%PMPTraj 	robot_trajectory~%~%================================================================================~%MSG: robot_serving/PMPTraj~%# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Movement-response)))
  "Returns full string definition for message of type 'Movement-response"
  (cl:format cl:nil "~%PMPTraj 	robot_trajectory~%~%================================================================================~%MSG: robot_serving/PMPTraj~%# ROS message for a PMP trajectory. each entry of traj and time_step must have the same length~%PMPPoint[] 	traj		# vector, of the same size as the robot's DOFs, with the sequence of joint values at each time step for each DOF.~%float64[] 	time_step	# times of observations, in seconds, starting at zero~%================================================================================~%MSG: robot_serving/PMPPoint~%# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Movement-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Movement-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Movement-response
    (cl:cons ':robot_trajectory (robot_trajectory msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Movement)))
  'Movement-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Movement)))
  'Movement-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Movement)))
  "Returns string type for a service object of type '<Movement>"
  "robot_serving/Movement")