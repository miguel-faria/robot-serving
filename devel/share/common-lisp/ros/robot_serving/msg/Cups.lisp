; Auto-generated. Do not edit!


(cl:in-package robot_serving-msg)


;//! \htmlinclude Cups.msg.html

(cl:defclass <Cups> (roslisp-msg-protocol:ros-message)
  ((n_cups
    :reader n_cups
    :initarg :n_cups
    :type cl:integer
    :initform 0)
   (cups_color
    :reader cups_color
    :initarg :cups_color
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (cups_pos_x
    :reader cups_pos_x
    :initarg :cups_pos_x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (cups_pos_y
    :reader cups_pos_y
    :initarg :cups_pos_y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (cups_pos_z
    :reader cups_pos_z
    :initarg :cups_pos_z
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (depth_width
    :reader depth_width
    :initarg :depth_width
    :type cl:integer
    :initform 0)
   (depth_height
    :reader depth_height
    :initarg :depth_height
    :type cl:integer
    :initform 0))
)

(cl:defclass Cups (<Cups>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cups>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cups)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_serving-msg:<Cups> is deprecated: use robot_serving-msg:Cups instead.")))

(cl:ensure-generic-function 'n_cups-val :lambda-list '(m))
(cl:defmethod n_cups-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:n_cups-val is deprecated.  Use robot_serving-msg:n_cups instead.")
  (n_cups m))

(cl:ensure-generic-function 'cups_color-val :lambda-list '(m))
(cl:defmethod cups_color-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:cups_color-val is deprecated.  Use robot_serving-msg:cups_color instead.")
  (cups_color m))

(cl:ensure-generic-function 'cups_pos_x-val :lambda-list '(m))
(cl:defmethod cups_pos_x-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:cups_pos_x-val is deprecated.  Use robot_serving-msg:cups_pos_x instead.")
  (cups_pos_x m))

(cl:ensure-generic-function 'cups_pos_y-val :lambda-list '(m))
(cl:defmethod cups_pos_y-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:cups_pos_y-val is deprecated.  Use robot_serving-msg:cups_pos_y instead.")
  (cups_pos_y m))

(cl:ensure-generic-function 'cups_pos_z-val :lambda-list '(m))
(cl:defmethod cups_pos_z-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:cups_pos_z-val is deprecated.  Use robot_serving-msg:cups_pos_z instead.")
  (cups_pos_z m))

(cl:ensure-generic-function 'depth_width-val :lambda-list '(m))
(cl:defmethod depth_width-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:depth_width-val is deprecated.  Use robot_serving-msg:depth_width instead.")
  (depth_width m))

(cl:ensure-generic-function 'depth_height-val :lambda-list '(m))
(cl:defmethod depth_height-val ((m <Cups>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_serving-msg:depth_height-val is deprecated.  Use robot_serving-msg:depth_height instead.")
  (depth_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cups>) ostream)
  "Serializes a message object of type '<Cups>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_cups)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_cups)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n_cups)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n_cups)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cups_color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'cups_color))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cups_pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cups_pos_x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cups_pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cups_pos_y))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cups_pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'cups_pos_z))
  (cl:let* ((signed (cl:slot-value msg 'depth_width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'depth_height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cups>) istream)
  "Deserializes a message object of type '<Cups>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_cups)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_cups)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n_cups)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n_cups)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cups_color) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cups_color)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cups_pos_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cups_pos_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cups_pos_y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cups_pos_y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cups_pos_z) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cups_pos_z)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cups>)))
  "Returns string type for a message object of type '<Cups>"
  "robot_serving/Cups")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cups)))
  "Returns string type for a message object of type 'Cups"
  "robot_serving/Cups")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cups>)))
  "Returns md5sum for a message object of type '<Cups>"
  "3a9d0ae4758d926f4f7c5e0d4fe36472")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cups)))
  "Returns md5sum for a message object of type 'Cups"
  "3a9d0ae4758d926f4f7c5e0d4fe36472")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cups>)))
  "Returns full string definition for message of type '<Cups>"
  (cl:format cl:nil "# This message contains information about the position of n colored cups~%uint32 		n_cups 		#number of cups recorded in the workspace~%string[] 	cups_color 	#vector with the designations given to each cup~%float32[] 	cups_pos_x 	#this is a n matrix with the x position of each cup (in millimeters)~%float32[] 	cups_pos_y 	#this is a n matrix with the y position of each cup (in millimeters)~%float32[] 	cups_pos_z 	#this is a n matrix with the z position of each cup (in millimeters)~%int32 		depth_width 	#this is the width of the depth image (in pixels)~%int32 		depth_height 	#this is the height of the depth image (in pixels) ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cups)))
  "Returns full string definition for message of type 'Cups"
  (cl:format cl:nil "# This message contains information about the position of n colored cups~%uint32 		n_cups 		#number of cups recorded in the workspace~%string[] 	cups_color 	#vector with the designations given to each cup~%float32[] 	cups_pos_x 	#this is a n matrix with the x position of each cup (in millimeters)~%float32[] 	cups_pos_y 	#this is a n matrix with the y position of each cup (in millimeters)~%float32[] 	cups_pos_z 	#this is a n matrix with the z position of each cup (in millimeters)~%int32 		depth_width 	#this is the width of the depth image (in pixels)~%int32 		depth_height 	#this is the height of the depth image (in pixels) ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cups>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cups_color) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cups_pos_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cups_pos_y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cups_pos_z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cups>))
  "Converts a ROS message object to a list"
  (cl:list 'Cups
    (cl:cons ':n_cups (n_cups msg))
    (cl:cons ':cups_color (cups_color msg))
    (cl:cons ':cups_pos_x (cups_pos_x msg))
    (cl:cons ':cups_pos_y (cups_pos_y msg))
    (cl:cons ':cups_pos_z (cups_pos_z msg))
    (cl:cons ':depth_width (depth_width msg))
    (cl:cons ':depth_height (depth_height msg))
))
