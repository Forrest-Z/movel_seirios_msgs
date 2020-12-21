; Auto-generated. Do not edit!


(cl:in-package cob_map_accessibility_analysis-srv)


;//! \htmlinclude CheckPerimeterAccessibility-request.msg.html

(cl:defclass <CheckPerimeterAccessibility-request> (roslisp-msg-protocol:ros-message)
  ((center
    :reader center
    :initarg :center
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (rotational_sampling_step
    :reader rotational_sampling_step
    :initarg :rotational_sampling_step
    :type cl:float
    :initform 0.0))
)

(cl:defclass CheckPerimeterAccessibility-request (<CheckPerimeterAccessibility-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPerimeterAccessibility-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPerimeterAccessibility-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_map_accessibility_analysis-srv:<CheckPerimeterAccessibility-request> is deprecated: use cob_map_accessibility_analysis-srv:CheckPerimeterAccessibility-request instead.")))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <CheckPerimeterAccessibility-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:center-val is deprecated.  Use cob_map_accessibility_analysis-srv:center instead.")
  (center m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <CheckPerimeterAccessibility-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:radius-val is deprecated.  Use cob_map_accessibility_analysis-srv:radius instead.")
  (radius m))

(cl:ensure-generic-function 'rotational_sampling_step-val :lambda-list '(m))
(cl:defmethod rotational_sampling_step-val ((m <CheckPerimeterAccessibility-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:rotational_sampling_step-val is deprecated.  Use cob_map_accessibility_analysis-srv:rotational_sampling_step instead.")
  (rotational_sampling_step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPerimeterAccessibility-request>) ostream)
  "Serializes a message object of type '<CheckPerimeterAccessibility-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotational_sampling_step))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPerimeterAccessibility-request>) istream)
  "Deserializes a message object of type '<CheckPerimeterAccessibility-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotational_sampling_step) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPerimeterAccessibility-request>)))
  "Returns string type for a service object of type '<CheckPerimeterAccessibility-request>"
  "cob_map_accessibility_analysis/CheckPerimeterAccessibilityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPerimeterAccessibility-request)))
  "Returns string type for a service object of type 'CheckPerimeterAccessibility-request"
  "cob_map_accessibility_analysis/CheckPerimeterAccessibilityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPerimeterAccessibility-request>)))
  "Returns md5sum for a message object of type '<CheckPerimeterAccessibility-request>"
  "26a8e5959c459eafca624877deadd8ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPerimeterAccessibility-request)))
  "Returns md5sum for a message object of type 'CheckPerimeterAccessibility-request"
  "26a8e5959c459eafca624877deadd8ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPerimeterAccessibility-request>)))
  "Returns full string definition for message of type '<CheckPerimeterAccessibility-request>"
  (cl:format cl:nil "geometry_msgs/Pose2D center       # center of the circle whose perimeter should be checked for accessibility, in [m,m,rad]~%float64 radius                    # radius of the circle, in [m]~%float64 rotational_sampling_step  # rotational sampling step width for checking points on the perimeter, in [rad] ~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPerimeterAccessibility-request)))
  "Returns full string definition for message of type 'CheckPerimeterAccessibility-request"
  (cl:format cl:nil "geometry_msgs/Pose2D center       # center of the circle whose perimeter should be checked for accessibility, in [m,m,rad]~%float64 radius                    # radius of the circle, in [m]~%float64 rotational_sampling_step  # rotational sampling step width for checking points on the perimeter, in [rad] ~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPerimeterAccessibility-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPerimeterAccessibility-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPerimeterAccessibility-request
    (cl:cons ':center (center msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':rotational_sampling_step (rotational_sampling_step msg))
))
;//! \htmlinclude CheckPerimeterAccessibility-response.msg.html

(cl:defclass <CheckPerimeterAccessibility-response> (roslisp-msg-protocol:ros-message)
  ((accessible_poses_on_perimeter
    :reader accessible_poses_on_perimeter
    :initarg :accessible_poses_on_perimeter
    :type (cl:vector geometry_msgs-msg:Pose2D)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose2D :initial-element (cl:make-instance 'geometry_msgs-msg:Pose2D))))
)

(cl:defclass CheckPerimeterAccessibility-response (<CheckPerimeterAccessibility-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckPerimeterAccessibility-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckPerimeterAccessibility-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_map_accessibility_analysis-srv:<CheckPerimeterAccessibility-response> is deprecated: use cob_map_accessibility_analysis-srv:CheckPerimeterAccessibility-response instead.")))

(cl:ensure-generic-function 'accessible_poses_on_perimeter-val :lambda-list '(m))
(cl:defmethod accessible_poses_on_perimeter-val ((m <CheckPerimeterAccessibility-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_map_accessibility_analysis-srv:accessible_poses_on_perimeter-val is deprecated.  Use cob_map_accessibility_analysis-srv:accessible_poses_on_perimeter instead.")
  (accessible_poses_on_perimeter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckPerimeterAccessibility-response>) ostream)
  "Serializes a message object of type '<CheckPerimeterAccessibility-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'accessible_poses_on_perimeter))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'accessible_poses_on_perimeter))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckPerimeterAccessibility-response>) istream)
  "Deserializes a message object of type '<CheckPerimeterAccessibility-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'accessible_poses_on_perimeter) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'accessible_poses_on_perimeter)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckPerimeterAccessibility-response>)))
  "Returns string type for a service object of type '<CheckPerimeterAccessibility-response>"
  "cob_map_accessibility_analysis/CheckPerimeterAccessibilityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPerimeterAccessibility-response)))
  "Returns string type for a service object of type 'CheckPerimeterAccessibility-response"
  "cob_map_accessibility_analysis/CheckPerimeterAccessibilityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckPerimeterAccessibility-response>)))
  "Returns md5sum for a message object of type '<CheckPerimeterAccessibility-response>"
  "26a8e5959c459eafca624877deadd8ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckPerimeterAccessibility-response)))
  "Returns md5sum for a message object of type 'CheckPerimeterAccessibility-response"
  "26a8e5959c459eafca624877deadd8ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckPerimeterAccessibility-response>)))
  "Returns full string definition for message of type '<CheckPerimeterAccessibility-response>"
  (cl:format cl:nil "geometry_msgs/Pose2D[] accessible_poses_on_perimeter 	  # array of accessible poses on the perimeter of the circle~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckPerimeterAccessibility-response)))
  "Returns full string definition for message of type 'CheckPerimeterAccessibility-response"
  (cl:format cl:nil "geometry_msgs/Pose2D[] accessible_poses_on_perimeter 	  # array of accessible poses on the perimeter of the circle~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckPerimeterAccessibility-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'accessible_poses_on_perimeter) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckPerimeterAccessibility-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckPerimeterAccessibility-response
    (cl:cons ':accessible_poses_on_perimeter (accessible_poses_on_perimeter msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckPerimeterAccessibility)))
  'CheckPerimeterAccessibility-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckPerimeterAccessibility)))
  'CheckPerimeterAccessibility-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckPerimeterAccessibility)))
  "Returns string type for a service object of type '<CheckPerimeterAccessibility>"
  "cob_map_accessibility_analysis/CheckPerimeterAccessibility")