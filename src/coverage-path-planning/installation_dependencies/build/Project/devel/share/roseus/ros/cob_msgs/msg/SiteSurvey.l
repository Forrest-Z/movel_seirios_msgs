;; Auto-generated. Do not edit!


(when (boundp 'cob_msgs::SiteSurvey)
  (if (not (find-package "COB_MSGS"))
    (make-package "COB_MSGS"))
  (shadow 'SiteSurvey (find-package "COB_MSGS")))
(unless (find-package "COB_MSGS::SITESURVEY")
  (make-package "COB_MSGS::SITESURVEY"))

(in-package "ROS")
;;//! \htmlinclude SiteSurvey.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_msgs::SiteSurvey
  :super ros::object
  :slots (_header _networks ))

(defmethod cob_msgs::SiteSurvey
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:networks __networks) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _networks __networks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:networks
   (&rest __networks)
   (if (keywordp (car __networks))
       (send* _networks __networks)
     (progn
       (if __networks (setq _networks (car __networks)))
       _networks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; cob_msgs/Network[] _networks
    (apply #'+ (send-all _networks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; cob_msgs/Network[] _networks
     (write-long (length _networks) s)
     (dolist (elem _networks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; cob_msgs/Network[] _networks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _networks (let (r) (dotimes (i n) (push (instance cob_msgs::Network :init) r)) r))
     (dolist (elem- _networks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cob_msgs::SiteSurvey :md5sum-) "f1063b16bb121ef190ae5edfe09d94ec")
(setf (get cob_msgs::SiteSurvey :datatype-) "cob_msgs/SiteSurvey")
(setf (get cob_msgs::SiteSurvey :definition-)
      "Header header
Network[] networks

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: cob_msgs/Network
string macattr
string essid
int32 channel
int32 rssi
int32 noise
int32 beacon

")



(provide :cob_msgs/SiteSurvey "f1063b16bb121ef190ae5edfe09d94ec")


