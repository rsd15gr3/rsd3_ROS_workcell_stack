; Auto-generated. Do not edit!


(cl:in-package wsg_50_common-srv)


;//! \htmlinclude Conf-request.msg.html

(cl:defclass <Conf-request> (roslisp-msg-protocol:ros-message)
  ((val
    :reader val
    :initarg :val
    :type cl:float
    :initform 0.0))
)

(cl:defclass Conf-request (<Conf-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Conf-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Conf-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wsg_50_common-srv:<Conf-request> is deprecated: use wsg_50_common-srv:Conf-request instead.")))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <Conf-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wsg_50_common-srv:val-val is deprecated.  Use wsg_50_common-srv:val instead.")
  (val m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Conf-request>) ostream)
  "Serializes a message object of type '<Conf-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Conf-request>) istream)
  "Deserializes a message object of type '<Conf-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'val) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Conf-request>)))
  "Returns string type for a service object of type '<Conf-request>"
  "wsg_50_common/ConfRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Conf-request)))
  "Returns string type for a service object of type 'Conf-request"
  "wsg_50_common/ConfRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Conf-request>)))
  "Returns md5sum for a message object of type '<Conf-request>"
  "0c7a1839200830facdc1205cf36615d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Conf-request)))
  "Returns md5sum for a message object of type 'Conf-request"
  "0c7a1839200830facdc1205cf36615d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Conf-request>)))
  "Returns full string definition for message of type '<Conf-request>"
  (cl:format cl:nil "float32 val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Conf-request)))
  "Returns full string definition for message of type 'Conf-request"
  (cl:format cl:nil "float32 val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Conf-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Conf-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Conf-request
    (cl:cons ':val (val msg))
))
;//! \htmlinclude Conf-response.msg.html

(cl:defclass <Conf-response> (roslisp-msg-protocol:ros-message)
  ((error
    :reader error
    :initarg :error
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Conf-response (<Conf-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Conf-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Conf-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wsg_50_common-srv:<Conf-response> is deprecated: use wsg_50_common-srv:Conf-response instead.")))

(cl:ensure-generic-function 'error-val :lambda-list '(m))
(cl:defmethod error-val ((m <Conf-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wsg_50_common-srv:error-val is deprecated.  Use wsg_50_common-srv:error instead.")
  (error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Conf-response>) ostream)
  "Serializes a message object of type '<Conf-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Conf-response>) istream)
  "Deserializes a message object of type '<Conf-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'error)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Conf-response>)))
  "Returns string type for a service object of type '<Conf-response>"
  "wsg_50_common/ConfResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Conf-response)))
  "Returns string type for a service object of type 'Conf-response"
  "wsg_50_common/ConfResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Conf-response>)))
  "Returns md5sum for a message object of type '<Conf-response>"
  "0c7a1839200830facdc1205cf36615d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Conf-response)))
  "Returns md5sum for a message object of type 'Conf-response"
  "0c7a1839200830facdc1205cf36615d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Conf-response>)))
  "Returns full string definition for message of type '<Conf-response>"
  (cl:format cl:nil "uint8 error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Conf-response)))
  "Returns full string definition for message of type 'Conf-response"
  (cl:format cl:nil "uint8 error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Conf-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Conf-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Conf-response
    (cl:cons ':error (error msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Conf)))
  'Conf-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Conf)))
  'Conf-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Conf)))
  "Returns string type for a service object of type '<Conf>"
  "wsg_50_common/Conf")