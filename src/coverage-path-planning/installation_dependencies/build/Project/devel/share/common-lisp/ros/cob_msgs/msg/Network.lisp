; Auto-generated. Do not edit!


(cl:in-package cob_msgs-msg)


;//! \htmlinclude Network.msg.html

(cl:defclass <Network> (roslisp-msg-protocol:ros-message)
  ((macattr
    :reader macattr
    :initarg :macattr
    :type cl:string
    :initform "")
   (essid
    :reader essid
    :initarg :essid
    :type cl:string
    :initform "")
   (channel
    :reader channel
    :initarg :channel
    :type cl:integer
    :initform 0)
   (rssi
    :reader rssi
    :initarg :rssi
    :type cl:integer
    :initform 0)
   (noise
    :reader noise
    :initarg :noise
    :type cl:integer
    :initform 0)
   (beacon
    :reader beacon
    :initarg :beacon
    :type cl:integer
    :initform 0))
)

(cl:defclass Network (<Network>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Network>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Network)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_msgs-msg:<Network> is deprecated: use cob_msgs-msg:Network instead.")))

(cl:ensure-generic-function 'macattr-val :lambda-list '(m))
(cl:defmethod macattr-val ((m <Network>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:macattr-val is deprecated.  Use cob_msgs-msg:macattr instead.")
  (macattr m))

(cl:ensure-generic-function 'essid-val :lambda-list '(m))
(cl:defmethod essid-val ((m <Network>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:essid-val is deprecated.  Use cob_msgs-msg:essid instead.")
  (essid m))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <Network>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:channel-val is deprecated.  Use cob_msgs-msg:channel instead.")
  (channel m))

(cl:ensure-generic-function 'rssi-val :lambda-list '(m))
(cl:defmethod rssi-val ((m <Network>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:rssi-val is deprecated.  Use cob_msgs-msg:rssi instead.")
  (rssi m))

(cl:ensure-generic-function 'noise-val :lambda-list '(m))
(cl:defmethod noise-val ((m <Network>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:noise-val is deprecated.  Use cob_msgs-msg:noise instead.")
  (noise m))

(cl:ensure-generic-function 'beacon-val :lambda-list '(m))
(cl:defmethod beacon-val ((m <Network>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:beacon-val is deprecated.  Use cob_msgs-msg:beacon instead.")
  (beacon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Network>) ostream)
  "Serializes a message object of type '<Network>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'macattr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'macattr))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'essid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'essid))
  (cl:let* ((signed (cl:slot-value msg 'channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rssi)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'noise)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'beacon)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Network>) istream)
  "Deserializes a message object of type '<Network>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'macattr) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'macattr) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'essid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'essid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rssi) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'noise) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'beacon) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Network>)))
  "Returns string type for a message object of type '<Network>"
  "cob_msgs/Network")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Network)))
  "Returns string type for a message object of type 'Network"
  "cob_msgs/Network")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Network>)))
  "Returns md5sum for a message object of type '<Network>"
  "b0854419660dc197dd94305843bee07f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Network)))
  "Returns md5sum for a message object of type 'Network"
  "b0854419660dc197dd94305843bee07f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Network>)))
  "Returns full string definition for message of type '<Network>"
  (cl:format cl:nil "string macattr~%string essid~%int32 channel~%int32 rssi~%int32 noise~%int32 beacon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Network)))
  "Returns full string definition for message of type 'Network"
  (cl:format cl:nil "string macattr~%string essid~%int32 channel~%int32 rssi~%int32 noise~%int32 beacon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Network>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'macattr))
     4 (cl:length (cl:slot-value msg 'essid))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Network>))
  "Converts a ROS message object to a list"
  (cl:list 'Network
    (cl:cons ':macattr (macattr msg))
    (cl:cons ':essid (essid msg))
    (cl:cons ':channel (channel msg))
    (cl:cons ':rssi (rssi msg))
    (cl:cons ':noise (noise msg))
    (cl:cons ':beacon (beacon msg))
))
