; Auto-generated. Do not edit!


(cl:in-package robot_serving-srv)


;//! \htmlinclude RobotMovementFeedback-request.msg.html

(cl:defclass <RobotMovementFeedback-request> (roslisp-msg-protocol:ros-message)
  ((limb
    :reader limb
    :initarg :limb
    :type cl:string
    :initform ""))
)

(cl:defclass RobotMovementFeedback-request (<RobotMovementFeedback-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMovementFeedback-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMovementFeedback-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<RobotMovementFeedback-request> is deprecated: use robot_serving-srv:RobotMovementFeedback-request instead.")))

(cl:ensure-generic-function 'limb-val :lambda-list '(m))
(cl:defmethod limb-val ((m <RobotMovementFeedback-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:limb-val is deprecated.  Use robot_serving-srv:limb instead.")
  (limb m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMovementFeedback-request>) ostream)
  "Serializes a message object of type '<RobotMovementFeedback-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'limb))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'limb))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMovementFeedback-request>) istream)
  "Deserializes a message object of type '<RobotMovementFeedback-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'limb) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'limb) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMovementFeedback-request>)))
  "Returns string type for a service object of type '<RobotMovementFeedback-request>"
  "robot_serving/RobotMovementFeedbackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementFeedback-request)))
  "Returns string type for a service object of type 'RobotMovementFeedback-request"
  "robot_serving/RobotMovementFeedbackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMovementFeedback-request>)))
  "Returns md5sum for a message object of type '<RobotMovementFeedback-request>"
  "42c90eeae941df68ef24a656cdcd8c94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMovementFeedback-request)))
  "Returns md5sum for a message object of type 'RobotMovementFeedback-request"
  "42c90eeae941df68ef24a656cdcd8c94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMovementFeedback-request>)))
  "Returns full string definition for message of type '<RobotMovementFeedback-request>"
  (cl:format cl:nil "~%string 	limb~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMovementFeedback-request)))
  "Returns full string definition for message of type 'RobotMovementFeedback-request"
  (cl:format cl:nil "~%string 	limb~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMovementFeedback-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'limb))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMovementFeedback-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMovementFeedback-request
    (cl:cons ':limb (limb msg))
))
;//! \htmlinclude RobotMovementFeedback-response.msg.html

(cl:defclass <RobotMovementFeedback-response> (roslisp-msg-protocol:ros-message)
  ((feedback
    :reader feedback
    :initarg :feedback
    :type cl:integer
    :initform 0))
)

(cl:defclass RobotMovementFeedback-response (<RobotMovementFeedback-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotMovementFeedback-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotMovementFeedback-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-srv:<RobotMovementFeedback-response> is deprecated: use robot_serving-srv:RobotMovementFeedback-response instead.")))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <RobotMovementFeedback-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-srv:feedback-val is deprecated.  Use robot_serving-srv:feedback instead.")
  (feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotMovementFeedback-response>) ostream)
  "Serializes a message object of type '<RobotMovementFeedback-response>"
  (cl:let* ((signed (cl:slot-value msg 'feedback)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotMovementFeedback-response>) istream)
  "Deserializes a message object of type '<RobotMovementFeedback-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feedback) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotMovementFeedback-response>)))
  "Returns string type for a service object of type '<RobotMovementFeedback-response>"
  "robot_serving/RobotMovementFeedbackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementFeedback-response)))
  "Returns string type for a service object of type 'RobotMovementFeedback-response"
  "robot_serving/RobotMovementFeedbackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotMovementFeedback-response>)))
  "Returns md5sum for a message object of type '<RobotMovementFeedback-response>"
  "42c90eeae941df68ef24a656cdcd8c94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotMovementFeedback-response)))
  "Returns md5sum for a message object of type 'RobotMovementFeedback-response"
  "42c90eeae941df68ef24a656cdcd8c94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotMovementFeedback-response>)))
  "Returns full string definition for message of type '<RobotMovementFeedback-response>"
  (cl:format cl:nil "~%int32 	feedback~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotMovementFeedback-response)))
  "Returns full string definition for message of type 'RobotMovementFeedback-response"
  (cl:format cl:nil "~%int32 	feedback~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotMovementFeedback-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotMovementFeedback-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotMovementFeedback-response
    (cl:cons ':feedback (feedback msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RobotMovementFeedback)))
  'RobotMovementFeedback-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RobotMovementFeedback)))
  'RobotMovementFeedback-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotMovementFeedback)))
  "Returns string type for a service object of type '<RobotMovementFeedback>"
  "robot_serving/RobotMovementFeedback")