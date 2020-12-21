;; Auto-generated. Do not edit!


(when (boundp 'cob_map_accessibility_analysis::CheckPerimeterAccessibility)
  (if (not (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS"))
    (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS"))
  (shadow 'CheckPerimeterAccessibility (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS")))
(unless (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPERIMETERACCESSIBILITY")
  (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPERIMETERACCESSIBILITY"))
(unless (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPERIMETERACCESSIBILITYREQUEST")
  (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPERIMETERACCESSIBILITYREQUEST"))
(unless (find-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPERIMETERACCESSIBILITYRESPONSE")
  (make-package "COB_MAP_ACCESSIBILITY_ANALYSIS::CHECKPERIMETERACCESSIBILITYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest
  :super ros::object
  :slots (_center _radius _rotational_sampling_step ))

(defmethod cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest
  (:init
   (&key
    ((:center __center) (instance geometry_msgs::Pose2D :init))
    ((:radius __radius) 0.0)
    ((:rotational_sampling_step __rotational_sampling_step) 0.0)
    )
   (send-super :init)
   (setq _center __center)
   (setq _radius (float __radius))
   (setq _rotational_sampling_step (float __rotational_sampling_step))
   self)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:rotational_sampling_step
   (&optional __rotational_sampling_step)
   (if __rotational_sampling_step (setq _rotational_sampling_step __rotational_sampling_step)) _rotational_sampling_step)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D _center
    (send _center :serialization-length)
    ;; float64 _radius
    8
    ;; float64 _rotational_sampling_step
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D _center
       (send _center :serialize s)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rotational_sampling_step
       (sys::poke _rotational_sampling_step (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rotational_sampling_step
     (setq _rotational_sampling_step (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse
  :super ros::object
  :slots (_accessible_poses_on_perimeter ))

(defmethod cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse
  (:init
   (&key
    ((:accessible_poses_on_perimeter __accessible_poses_on_perimeter) ())
    )
   (send-super :init)
   (setq _accessible_poses_on_perimeter __accessible_poses_on_perimeter)
   self)
  (:accessible_poses_on_perimeter
   (&rest __accessible_poses_on_perimeter)
   (if (keywordp (car __accessible_poses_on_perimeter))
       (send* _accessible_poses_on_perimeter __accessible_poses_on_perimeter)
     (progn
       (if __accessible_poses_on_perimeter (setq _accessible_poses_on_perimeter (car __accessible_poses_on_perimeter)))
       _accessible_poses_on_perimeter)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose2D[] _accessible_poses_on_perimeter
    (apply #'+ (send-all _accessible_poses_on_perimeter :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose2D[] _accessible_poses_on_perimeter
     (write-long (length _accessible_poses_on_perimeter) s)
     (dolist (elem _accessible_poses_on_perimeter)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose2D[] _accessible_poses_on_perimeter
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accessible_poses_on_perimeter (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose2D :init) r)) r))
     (dolist (elem- _accessible_poses_on_perimeter)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass cob_map_accessibility_analysis::CheckPerimeterAccessibility
  :super ros::object
  :slots ())

(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibility :md5sum-) "26a8e5959c459eafca624877deadd8ec")
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibility :datatype-) "cob_map_accessibility_analysis/CheckPerimeterAccessibility")
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibility :request) cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest)
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibility :response) cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse)

(defmethod cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest
  (:response () (instance cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse :init)))

(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest :md5sum-) "26a8e5959c459eafca624877deadd8ec")
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest :datatype-) "cob_map_accessibility_analysis/CheckPerimeterAccessibilityRequest")
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibilityRequest :definition-)
      "geometry_msgs/Pose2D center       # center of the circle whose perimeter should be checked for accessibility, in [m,m,rad]
float64 radius                    # radius of the circle, in [m]
float64 rotational_sampling_step  # rotational sampling step width for checking points on the perimeter, in [rad] 

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
geometry_msgs/Pose2D[] accessible_poses_on_perimeter 	  # array of accessible poses on the perimeter of the circle

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
")

(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse :md5sum-) "26a8e5959c459eafca624877deadd8ec")
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse :datatype-) "cob_map_accessibility_analysis/CheckPerimeterAccessibilityResponse")
(setf (get cob_map_accessibility_analysis::CheckPerimeterAccessibilityResponse :definition-)
      "geometry_msgs/Pose2D center       # center of the circle whose perimeter should be checked for accessibility, in [m,m,rad]
float64 radius                    # radius of the circle, in [m]
float64 rotational_sampling_step  # rotational sampling step width for checking points on the perimeter, in [rad] 

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
geometry_msgs/Pose2D[] accessible_poses_on_perimeter 	  # array of accessible poses on the perimeter of the circle

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
")



(provide :cob_map_accessibility_analysis/CheckPerimeterAccessibility "26a8e5959c459eafca624877deadd8ec")


