; Auto-generated. Do not edit!


(cl:in-package robot_serving-msg)


;//! \htmlinclude PMPPoint.msg.html

(cl:defclass <PMPPoint> (roslisp-msg-protocol:ros-message)
  ((joint_angles
    :reader joint_angles
    :initarg :joint_angles
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PMPPoint (<PMPPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PMPPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PMPPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-msg:<PMPPoint> is deprecated: use robot_serving-msg:PMPPoint instead.")))

(cl:ensure-generic-function 'joint_angles-val :lambda-list '(m))
(cl:defmethod joint_angles-val ((m <PMPPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:joint_angles-val is deprecated.  Use robot_serving-msg:joint_angles instead.")
  (joint_angles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PMPPoint>) ostream)
  "Serializes a message object of type '<PMPPoint>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_angles))))
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
   (cl:slot-value msg 'joint_angles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PMPPoint>) istream)
  "Deserializes a message object of type '<PMPPoint>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_angles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_angles)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PMPPoint>)))
  "Returns string type for a message object of type '<PMPPoint>"
  "robot_serving/PMPPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PMPPoint)))
  "Returns string type for a message object of type 'PMPPoint"
  "robot_serving/PMPPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PMPPoint>)))
  "Returns md5sum for a message object of type '<PMPPoint>"
  "9eebf9cc7d525d143ad033b65dacb648")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PMPPoint)))
  "Returns md5sum for a message object of type 'PMPPoint"
  "9eebf9cc7d525d143ad033b65dacb648")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PMPPoint>)))
  "Returns full string definition for message of type '<PMPPoint>"
  (cl:format cl:nil "# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PMPPoint)))
  "Returns full string definition for message of type 'PMPPoint"
  (cl:format cl:nil "# Value of joint angles for one DOF~%float64[] 	joint_angles 	# sequence of angle values for one joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PMPPoint>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_angles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PMPPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'PMPPoint
    (cl:cons ':joint_angles (joint_angles msg))
))
