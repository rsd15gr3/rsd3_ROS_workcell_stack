; Auto-generated. Do not edit!


(cl:in-package wsg_50_common-srv)


;//! \htmlinclude Incr-request.msg.html

(cl:defclass <Incr-request> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform "")
   (increment
    :reader increment
    :initarg :increment
    :type cl:float
    :initform 0.0))
)

(cl:defclass Incr-request (<Incr-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Incr-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Incr-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wsg_50_common-srv:<Incr-request> is deprecated: use wsg_50_common-srv:Incr-request instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Incr-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wsg_50_common-srv:direction-val is deprecated.  Use wsg_50_common-srv:direction instead.")
  (direction m))

(cl:ensure-generic-function 'increment-val :lambda-list '(m))
(cl:defmethod increment-val ((m <Incr-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wsg_50_common-srv:increment-val is deprecated.  Use wsg_50_common-srv:increment instead.")
  (increment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Incr-request>) ostream)
  "Serializes a message object of type '<Incr-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'increment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Incr-request>) istream)
  "Deserializes a message object of type '<Incr-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'increment) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Incr-request>)))
  "Returns string type for a service object of type '<Incr-request>"
  "wsg_50_common/IncrRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Incr-request)))
  "Returns string type for a service object of type 'Incr-request"
  "wsg_50_common/IncrRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Incr-request>)))
  "Returns md5sum for a message object of type '<Incr-request>"
  "b996830260cb95e784a9d9fbb5e34c61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Incr-request)))
  "Returns md5sum for a message object of type 'Incr-request"
  "b996830260cb95e784a9d9fbb5e34c61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Incr-request>)))
  "Returns full string definition for message of type '<Incr-request>"
  (cl:format cl:nil "string direction~%float32 increment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Incr-request)))
  "Returns full string definition for message of type 'Incr-request"
  (cl:format cl:nil "string direction~%float32 increment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Incr-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'direction))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Incr-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Incr-request
    (cl:cons ':direction (direction msg))
    (cl:cons ':increment (increment msg))
))
;//! \htmlinclude Incr-response.msg.html

(cl:defclass <Incr-response> (roslisp-msg-protocol:ros-message)
  ((error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Incr-response (<Incr-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Incr-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Incr-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wsg_50_common-srv:<Incr-response> is deprecated: use wsg_50_common-srv:Incr-response instead.")))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <Incr-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wsg_50_common-srv:error-val is deprecated.  Use wsg_50_common-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Incr-response>) ostream)
  "Serializes a message object of type '<Incr-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Incr-response>) istream)
  "Deserializes a message object of type '<Incr-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Incr-response>)))
  "Returns string type for a service object of type '<Incr-response>"
  "wsg_50_common/IncrResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Incr-response)))
  "Returns string type for a service object of type 'Incr-response"
  "wsg_50_common/IncrResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Incr-response>)))
  "Returns md5sum for a message object of type '<Incr-response>"
  "b996830260cb95e784a9d9fbb5e34c61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Incr-response)))
  "Returns md5sum for a message object of type 'Incr-response"
  "b996830260cb95e784a9d9fbb5e34c61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Incr-response>)))
  "Returns full string definition for message of type '<Incr-response>"
  (cl:format cl:nil "uint8 error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Incr-response)))
  "Returns full string definition for message of type 'Incr-response"
  (cl:format cl:nil "uint8 error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Incr-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Incr-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Incr-response
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Incr)))
  'Incr-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Incr)))
  'Incr-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Incr)))
  "Returns string type for a service object of type '<Incr>"
  "wsg_50_common/Incr")