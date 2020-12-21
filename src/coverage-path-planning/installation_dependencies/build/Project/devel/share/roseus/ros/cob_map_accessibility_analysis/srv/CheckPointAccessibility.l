;; Auto-generated. Do not edit!


(when (boundp 'cob_map_accessibility_analysis::CheckPointAccessibility)
  (if (not (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS"))
    (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS"))
  (shadow 'CheckPointAccessibility (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS")))
(unless (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPOINTACCESSIBILITY")
  (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPOINTACCESSIBILITY"))
(unless (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPOINTACCESSIBILITYREQUEST")
  (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPOINTACCESSIBILITYREQUEST"))
(unless (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPOINTACCESSIBILITYRESPONSE")
  (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPOINTACCESSIBILITYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass cob_map_accessibility_analysis::CheckPointAccessibilityRequest
  :super ros::object
  :slots (_points_to_check _approach_path_accessibility_check ))

(defmethod cob_map_accessibility_analysis::CheckPointAccessibilityRequest
  (:init
   (&key
    ((:points_to_check __points_to_check) ())
    ((:approach_path_accessibility_check __approach_path_accessibility_check) nil)
    )
   (send-super :init)
   (setq _points_to_check __points_to_check)
   (setq _approach_path_accessibility_check __approach_path_accessibility_check)
   self)
  (:points_to_check
   (&rest __points_to_check)
   (if (keywordp (car __points_to_check))
       (send* _points_to_check __points_to_check)
     (progn
       (if __points_to_check (setq _points_to_check (car __points_to_check)))
       _points_to_check)))
  (:approach_path_accessibility_check
   (&optional (__approach_path_accessibility_check :null))
   (if (not (eq __approach_path_accessibility_check :null)) (setq _approach_path_accessibility_check __approach_path_accessibility_check)) _approach_path_accessibility_check)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D[] _points_to_check
    (apply #'+ (send-all _points_to_check :serialization-length)) 4
    ;; bool _approach_path_accessibility_check
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D[] _points_to_check
     (write-long (length _points_to_check) s)
     (dolist (elem _points_to_check)
       (send elem :serialize s)
       )
     ;; bool _approach_path_accessibility_check
       (if _approach_path_accessibility_check (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D[] _points_to_check
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points_to_check (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose2D :init) r)) r))
     (dolist (elem- _points_to_check)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _approach_path_accessibility_check
     (setq _approach_path_accessibility_check (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass cob_map_accessibility_analysis::CheckPointAccessibilityResponse
  :super ros::object
  :slots (_accessibility_flags ))

(defmethod cob_map_accessibility_analysis::CheckPointAccessibilityResponse
  (:init
   (&key
    ((:accessibility_flags __accessibility_flags) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _accessibility_flags __accessibility_flags)
   self)
  (:accessibility_flags
   (&optional __accessibility_flags)
   (if __accessibility_flags (setq _accessibility_flags __accessibility_flags)) _accessibility_flags)
  (:serialization-length
   ()
   (+
    ;; bool[] _accessibility_flags
    (* 1    (length _accessibility_flags)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool[] _accessibility_flags
     (write-long (length _accessibility_flags) s)
     (dotimes (i (length _accessibility_flags))
       (if (elt _accessibility_flags i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool[] _accessibility_flags
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accessibility_flags (make-list n))
     (dotimes (i n)
     (setf (elt _accessibility_flags i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(defclass cob_map_accessibility_analysis::CheckPointAccessibility
  :super ros::object
  :slots ())

(setf (get cob_map_accessibility_analysis::CheckPointAccessibility :md5sum-) "c42c7449dffc73e50011cd7a1eb83e23")
(setf (get cob_map_accessibility_analysis::CheckPointAccessibility :datatype-) "cob_map_accessibility_analysis/CheckPointAccessibility")
(setf (get cob_map_accessibility_analysis::CheckPointAccessibility :request) cob_map_accessibility_analysis::CheckPointAccessibilityRequest)
(setf (get cob_map_accessibility_analysis::CheckPointAccessibility :response) cob_map_accessibility_analysis::CheckPointAccessibilityResponse)

(defmethod cob_map_accessibility_analysis::CheckPointAccessibilityRequest
  (:response () (instance cob_map_accessibility_analysis::CheckPointAccessibilityResponse :init)))

(setf (get cob_map_accessibility_analysis::CheckPointAccessibilityRequest :md5sum-) "c42c7449dffc73e50011cd7a1eb83e23")
(setf (get cob_map_accessibility_analysis::CheckPointAccessibilityRequest :datatype-) "cob_map_accessibility_analysis/CheckPointAccessibilityRequest")
(setf (get cob_map_accessibility_analysis::CheckPointAccessibilityRequest :definition-)
      "geometry_msgs/Pose2D[] points_to_check    # array of points which should be checked for accessibility
bool approach_path_accessibility_check    # if true, the path to a goal position must be accessible as well

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
---
bool[] accessibility_flags    			  # array of booleans which correspond to the points and define accessibility (true=free, false=obstacle)
")

(setf (get cob_map_accessibility_analysis::CheckPointAccessibilityResponse :md5sum-) "c42c7449dffc73e50011cd7a1eb83e23")
(setf (get cob_map_accessibility_analysis::CheckPointAccessibilityResponse :datatype-) "cob_map_accessibility_analysis/CheckPointAccessibilityResponse")
(setf (get cob_map_accessibility_analysis::CheckPointAccessibilityResponse :definition-)
      "geometry_msgs/Pose2D[] points_to_check    # array of points which should be checked for accessibility
bool approach_path_accessibility_check    # if true, the path to a goal position must be accessible as well

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
---
bool[] accessibility_flags    			  # array of booleans which correspond to the points and define accessibility (true=free, false=obstacle)
")



(provide :cob_map_accessibility_analysis/CheckPointAccessibility "c42c7449dffc73e50011cd7a1eb83e23")


