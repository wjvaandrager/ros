; Auto-generated. Do not edit!


(cl:in-package audioloc-msg)


;//! \htmlinclude MultiArrayFloat32.msg.html

(cl:defclass <MultiArrayFloat32> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:real
    :initform 0)
   (mic1
    :reader mic1
    :initarg :mic1
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mic2
    :reader mic2
    :initarg :mic2
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mic3
    :reader mic3
    :initarg :mic3
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mic4
    :reader mic4
    :initarg :mic4
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MultiArrayFloat32 (<MultiArrayFloat32>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiArrayFloat32>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiArrayFloat32)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audioloc-msg:<MultiArrayFloat32> is deprecated: use audioloc-msg:MultiArrayFloat32 instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <MultiArrayFloat32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audioloc-msg:data-val is deprecated.  Use audioloc-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'mic1-val :lambda-list '(m))
(cl:defmethod mic1-val ((m <MultiArrayFloat32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audioloc-msg:mic1-val is deprecated.  Use audioloc-msg:mic1 instead.")
  (mic1 m))

(cl:ensure-generic-function 'mic2-val :lambda-list '(m))
(cl:defmethod mic2-val ((m <MultiArrayFloat32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audioloc-msg:mic2-val is deprecated.  Use audioloc-msg:mic2 instead.")
  (mic2 m))

(cl:ensure-generic-function 'mic3-val :lambda-list '(m))
(cl:defmethod mic3-val ((m <MultiArrayFloat32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audioloc-msg:mic3-val is deprecated.  Use audioloc-msg:mic3 instead.")
  (mic3 m))

(cl:ensure-generic-function 'mic4-val :lambda-list '(m))
(cl:defmethod mic4-val ((m <MultiArrayFloat32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audioloc-msg:mic4-val is deprecated.  Use audioloc-msg:mic4 instead.")
  (mic4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiArrayFloat32>) ostream)
  "Serializes a message object of type '<MultiArrayFloat32>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'data)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'data) (cl:floor (cl:slot-value msg 'data)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mic1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mic1))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mic2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mic2))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mic3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mic3))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mic4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mic4))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiArrayFloat32>) istream)
  "Deserializes a message object of type '<MultiArrayFloat32>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mic1) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mic1)))
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
  (cl:setf (cl:slot-value msg 'mic2) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mic2)))
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
  (cl:setf (cl:slot-value msg 'mic3) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mic3)))
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
  (cl:setf (cl:slot-value msg 'mic4) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mic4)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiArrayFloat32>)))
  "Returns string type for a message object of type '<MultiArrayFloat32>"
  "audioloc/MultiArrayFloat32")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiArrayFloat32)))
  "Returns string type for a message object of type 'MultiArrayFloat32"
  "audioloc/MultiArrayFloat32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiArrayFloat32>)))
  "Returns md5sum for a message object of type '<MultiArrayFloat32>"
  "20fc89e40d151d6a5828699657cffb3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiArrayFloat32)))
  "Returns md5sum for a message object of type 'MultiArrayFloat32"
  "20fc89e40d151d6a5828699657cffb3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiArrayFloat32>)))
  "Returns full string definition for message of type '<MultiArrayFloat32>"
  (cl:format cl:nil "# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%time		  data~%float32[]         mic1          # array of mic1~%float32[]         mic2          # array of mic2~%float32[]         mic3          # array of mic3~%float32[]         mic4          # array of mic4~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiArrayFloat32)))
  "Returns full string definition for message of type 'MultiArrayFloat32"
  (cl:format cl:nil "# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%time		  data~%float32[]         mic1          # array of mic1~%float32[]         mic2          # array of mic2~%float32[]         mic3          # array of mic3~%float32[]         mic4          # array of mic4~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiArrayFloat32>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mic1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mic2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mic3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mic4) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiArrayFloat32>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiArrayFloat32
    (cl:cons ':data (data msg))
    (cl:cons ':mic1 (mic1 msg))
    (cl:cons ':mic2 (mic2 msg))
    (cl:cons ':mic3 (mic3 msg))
    (cl:cons ':mic4 (mic4 msg))
))
