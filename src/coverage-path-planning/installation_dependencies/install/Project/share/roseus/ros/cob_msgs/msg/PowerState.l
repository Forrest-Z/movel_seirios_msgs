;; Auto-generated. Do not edit!


(when (boundp 'cob_msgs::PowerState)
  (if (not (find-package "COB_MSGS"))
    (make-package "COB_MSGS"))
  (shadow 'PowerState (find-package "COB_MSGS")))
(unless (find-package "COB_MSGS::POWERSTATE")
  (make-package "COB_MSGS::POWERSTATE"))

(in-package "ROS")
;;//! \htmlinclude PowerState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass cob_msgs::PowerState
  :super ros::object
  :slots (_header _voltage _current _power_consumption _remaining_capacity _relative_remaining_capacity _charging _time_remaining _temperature ))

(defmethod cob_msgs::PowerState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:voltage __voltage) 0.0)
    ((:current __current) 0.0)
    ((:power_consumption __power_consumption) 0.0)
    ((:remaining_capacity __remaining_capacity) 0.0)
    ((:relative_remaining_capacity __relative_remaining_capacity) 0.0)
    ((:charging __charging) nil)
    ((:time_remaining __time_remaining) 0.0)
    ((:temperature __temperature) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _voltage (float __voltage))
   (setq _current (float __current))
   (setq _power_consumption (float __power_consumption))
   (setq _remaining_capacity (float __remaining_capacity))
   (setq _relative_remaining_capacity (float __relative_remaining_capacity))
   (setq _charging __charging)
   (setq _time_remaining (float __time_remaining))
   (setq _temperature (float __temperature))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:power_consumption
   (&optional __power_consumption)
   (if __power_consumption (setq _power_consumption __power_consumption)) _power_consumption)
  (:remaining_capacity
   (&optional __remaining_capacity)
   (if __remaining_capacity (setq _remaining_capacity __remaining_capacity)) _remaining_capacity)
  (:relative_remaining_capacity
   (&optional __relative_remaining_capacity)
   (if __relative_remaining_capacity (setq _relative_remaining_capacity __relative_remaining_capacity)) _relative_remaining_capacity)
  (:charging
   (&optional (__charging :null))
   (if (not (eq __charging :null)) (setq _charging __charging)) _charging)
  (:time_remaining
   (&optional __time_remaining)
   (if __time_remaining (setq _time_remaining __time_remaining)) _time_remaining)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _voltage
    8
    ;; float64 _current
    8
    ;; float64 _power_consumption
    8
    ;; float64 _remaining_capacity
    8
    ;; float64 _relative_remaining_capacity
    8
    ;; bool _charging
    1
    ;; float64 _time_remaining
    8
    ;; float64 _temperature
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _current
       (sys::poke _current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _power_consumption
       (sys::poke _power_consumption (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _remaining_capacity
       (sys::poke _remaining_capacity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _relative_remaining_capacity
       (sys::poke _relative_remaining_capacity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _charging
       (if _charging (write-byte -1 s) (write-byte 0 s))
     ;; float64 _time_remaining
       (sys::poke _time_remaining (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _current
     (setq _current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _power_consumption
     (setq _power_consumption (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _remaining_capacity
     (setq _remaining_capacity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _relative_remaining_capacity
     (setq _relative_remaining_capacity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _charging
     (setq _charging (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _time_remaining
     (setq _time_remaining (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _temperature
     (setq _temperature (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cob_msgs::PowerState :md5sum-) "70361e18e36187b93dab6d6ec8ca15e4")
(setf (get cob_msgs::PowerState :datatype-) "cob_msgs/PowerState")
(setf (get cob_msgs::PowerState :definition-)
      "# This message communicates the state of the power system.
Header header
float64 voltage                     # [V]
float64 current                     # [A]
float64 power_consumption           # [W] can only be calculated if not charging
float64 remaining_capacity          # [Ah]
float64 relative_remaining_capacity # [0..100] percent of maximum capacity (parameter max_capacity)
bool charging                       # flag if robot is connected to external power or not
float64 time_remaining              # [h] estimated time to empty or fully charged
float64 temperature                 # [Celsius] temperature of the battery

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

")



(provide :cob_msgs/PowerState "70361e18e36187b93dab6d6ec8ca15e4")


