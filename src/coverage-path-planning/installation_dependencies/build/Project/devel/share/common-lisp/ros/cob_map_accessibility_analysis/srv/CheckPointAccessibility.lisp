; Auto-generated. Do not edit!


(cl:in-package cob_map_accessibility_analysis-srv)


;//! \htmlinclude CheckPointAccessibility-request.msg.html

(cl:defclass <CheckPointAccessibility-request> (roslisp-msg-protocol:ros-message)
  ((points_to_check
    :reader points_to_check
    :initarg :points_to_check
    :type (cl:vector geometry_msgs-msg:Pose2D)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose2D :initial-element (cl:make-instance 'geometry_msgs-msg:Pose2D)))
   (approach_path_accessibility_check
    :reader approach_path_accessibility_check
    :initarg :approach_path_accessibility_check
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckPointAccessibility-request (<CheckPointAccessibility-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPointAccessibility-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPointAccessibility-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_map_accessibility_analysis-srv:<CheckPointAccessibility-request> is deprecated: use cob_map_accessibility_analysis-srv:CheckPointAccessibility-request instead.")))

(cl:ensure-generic-function 'points_to_check-val :lambda-list '(m))
(cl:defmethod points_to_check-val ((m <CheckPointAccessibility-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:points_to_check-val is deprecated.  Use cob_map_accessibility_analysis-srv:points_to_check instead.")
  (points_to_check m))

(cl:ensure-generic-function 'approach_path_accessibility_check-val :lambda-list '(m))
(cl:defmethod approach_path_accessibility_check-val ((m <CheckPointAccessibility-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:approach_path_accessibility_check-val is deprecated.  Use cob_map_accessibility_analysis-srv:approach_path_accessibility_check instead.")
  (approach_path_accessibility_check m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPointAccessibility-request>) ostream)
  "Serializes a message object of type '<CheckPointAccessibility-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points_to_check))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points_to_check))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'approach_path_accessibility_check) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPointAccessibility-request>) istream)
  "Deserializes a message object of type '<CheckPointAccessibility-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points_to_check) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points_to_check)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'approach_path_accessibility_check) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPointAccessibility-request>)))
  "Returns string type for a service object of type '<CheckPointAccessibility-request>"
  "cob_map_accessibility_analysis/CheckPointAccessibilityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPointAccessibility-request)))
  "Returns string type for a service object of type 'CheckPointAccessibility-request"
  "cob_map_accessibility_analysis/CheckPointAccessibilityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPointAccessibility-request>)))
  "Returns md5sum for a message object of type '<CheckPointAccessibility-request>"
  "c42c7449dffc73e50011cd7a1eb83e23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPointAccessibility-request)))
  "Returns md5sum for a message object of type 'CheckPointAccessibility-request"
  "c42c7449dffc73e50011cd7a1eb83e23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPointAccessibility-request>)))
  "Returns full string definition for message of type '<CheckPointAccessibility-request>"
  (cl:format cl:nil "geometry_msgs/Pose2D[] points_to_check    # array of points which should be checked for accessibility~%bool approach_path_accessibility_check    # if true, the path to a goal position must be accessible as well~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPointAccessibility-request)))
  "Returns full string definition for message of type 'CheckPointAccessibility-request"
  (cl:format cl:nil "geometry_msgs/Pose2D[] points_to_check    # array of points which should be checked for accessibility~%bool approach_path_accessibility_check    # if true, the path to a goal position must be accessible as well~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPointAccessibility-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points_to_check) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPointAccessibility-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPointAccessibility-request
    (cl:cons ':points_to_check (points_to_check msg))
    (cl:cons ':approach_path_accessibility_check (approach_path_accessibility_check msg))
))
;//! \htmlinclude CheckPointAccessibility-response.msg.html

(cl:defclass <CheckPointAccessibility-response> (roslisp-msg-protocol:ros-message)
  ((accessibility_flags
    :reader accessibility_flags
    :initarg :accessibility_flags
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass CheckPointAccessibility-response (<CheckPointAccessibility-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPointAccessibility-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPointAccessibility-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_map_accessibility_analysis-srv:<CheckPointAccessibility-response> is deprecated: use cob_map_accessibility_analysis-srv:CheckPointAccessibility-response instead.")))

(cl:ensure-generic-function 'accessibility_flags-val :lambda-list '(m))
(cl:defmethod accessibility_flags-val ((m <CheckPointAccessibility-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:accessibility_flags-val is deprecated.  Use cob_map_accessibility_analysis-srv:accessibility_flags instead.")
  (accessibility_flags m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPointAccessibility-response>) ostream)
  "Serializes a message object of type '<CheckPointAccessibility-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'accessibility_flags))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'accessibility_flags))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPointAccessibility-response>) istream)
  "Deserializes a message object of type '<CheckPointAccessibility-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'accessibility_flags) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'accessibility_flags)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPointAccessibility-response>)))
  "Returns string type for a service object of type '<CheckPointAccessibility-response>"
  "cob_map_accessibility_analysis/CheckPointAccessibilityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPointAccessibility-response)))
  "Returns string type for a service object of type 'CheckPointAccessibility-response"
  "cob_map_accessibility_analysis/CheckPointAccessibilityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPointAccessibility-response>)))
  "Returns md5sum for a message object of type '<CheckPointAccessibility-response>"
  "c42c7449dffc73e50011cd7a1eb83e23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPointAccessibility-response)))
  "Returns md5sum for a message object of type 'CheckPointAccessibility-response"
  "c42c7449dffc73e50011cd7a1eb83e23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPointAccessibility-response>)))
  "Returns full string definition for message of type '<CheckPointAccessibility-response>"
  (cl:format cl:nil "bool[] accessibility_flags    			  # array of booleans which correspond to the points and define accessibility (true=free, false=obstacle)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPointAccessibility-response)))
  "Returns full string definition for message of type 'CheckPointAccessibility-response"
  (cl:format cl:nil "bool[] accessibility_flags    			  # array of booleans which correspond to the points and define accessibility (true=free, false=obstacle)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPointAccessibility-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'accessibility_flags) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPointAccessibility-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPointAccessibility-response
    (cl:cons ':accessibility_flags (accessibility_flags msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPointAccessibility)))
  'CheckPointAccessibility-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPointAccessibility)))
  'CheckPointAccessibility-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPointAccessibility)))
  "Returns string type for a service object of type '<CheckPointAccessibility>"
  "cob_map_accessibility_analysis/CheckPointAccessibility")