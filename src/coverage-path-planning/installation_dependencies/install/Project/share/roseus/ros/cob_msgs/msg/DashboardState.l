;; Auto-generated. Do not edit!


(when (boundp 'cob_msgs::DashboardState)
  (if (not (find-package "COB_MSGS"))
    (make-package "COB_MSGS"))
  (shadow 'DashboardState (find-package "COB_MSGS")))
(unless (find-package "COB_MSGS::DASHBOARDSTATE")
  (make-package "COB_MSGS::DASHBOARDSTATE"))

(in-package "ROS")
;;//! \htmlinclude DashboardState.msg.html
(if (not (find-package "DIAGNOSTIC_MSGS"))
  (ros::roseus-add-msgs "diagnostic_msgs"))


(defclass cob_msgs::DashboardState
  :super ros::object
  :slots (_diagnostics_toplevel_state _power_state _emergency_stop_state ))

(defmethod cob_msgs::DashboardState
  (:init
   (&key
    ((:diagnostics_toplevel_state __diagnostics_toplevel_state) (instance diagnostic_msgs::DiagnosticStatus :init))
    ((:power_state __power_state) (instance cob_msgs::PowerState :init))
    ((:emergency_stop_state __emergency_stop_state) (instance cob_msgs::EmergencyStopState :init))
    )
   (send-super :init)
   (setq _diagnostics_toplevel_state __diagnostics_toplevel_state)
   (setq _power_state __power_state)
   (setq _emergency_stop_state __emergency_stop_state)
   self)
  (:diagnostics_toplevel_state
   (&rest __diagnostics_toplevel_state)
   (if (keywordp (car __diagnostics_toplevel_state))
       (send* _diagnostics_toplevel_state __diagnostics_toplevel_state)
     (progn
       (if __diagnostics_toplevel_state (setq _diagnostics_toplevel_state (car __diagnostics_toplevel_state)))
       _diagnostics_toplevel_state)))
  (:power_state
   (&rest __power_state)
   (if (keywordp (car __power_state))
       (send* _power_state __power_state)
     (progn
       (if __power_state (setq _power_state (car __power_state)))
       _power_state)))
  (:emergency_stop_state
   (&rest __emergency_stop_state)
   (if (keywordp (car __emergency_stop_state))
       (send* _emergency_stop_state __emergency_stop_state)
     (progn
       (if __emergency_stop_state (setq _emergency_stop_state (car __emergency_stop_state)))
       _emergency_stop_state)))
  (:serialization-length
   ()
   (+
    ;; diagnostic_msgs/DiagnosticStatus _diagnostics_toplevel_state
    (send _diagnostics_toplevel_state :serialization-length)
    ;; cob_msgs/PowerState _power_state
    (send _power_state :serialization-length)
    ;; cob_msgs/EmergencyStopState _emergency_stop_state
    (send _emergency_stop_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; diagnostic_msgs/DiagnosticStatus _diagnostics_toplevel_state
       (send _diagnostics_toplevel_state :serialize s)
     ;; cob_msgs/PowerState _power_state
       (send _power_state :serialize s)
     ;; cob_msgs/EmergencyStopState _emergency_stop_state
       (send _emergency_stop_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; diagnostic_msgs/DiagnosticStatus _diagnostics_toplevel_state
     (send _diagnostics_toplevel_state :deserialize buf ptr-) (incf ptr- (send _diagnostics_toplevel_state :serialization-length))
   ;; cob_msgs/PowerState _power_state
     (send _power_state :deserialize buf ptr-) (incf ptr- (send _power_state :serialization-length))
   ;; cob_msgs/EmergencyStopState _emergency_stop_state
     (send _emergency_stop_state :deserialize buf ptr-) (incf ptr- (send _emergency_stop_state :serialization-length))
   ;;
   self)
  )

(setf (get cob_msgs::DashboardState :md5sum-) "ac6926484e7f901e4a07c74a0bf08792")
(setf (get cob_msgs::DashboardState :datatype-) "cob_msgs/DashboardState")
(setf (get cob_msgs::DashboardState :definition-)
      "# This message communicates state information that might be used by a dashboard application.
diagnostic_msgs/DiagnosticStatus diagnostics_toplevel_state

cob_msgs/PowerState power_state

cob_msgs/EmergencyStopState emergency_stop_state

================================================================================
MSG: diagnostic_msgs/DiagnosticStatus
# This message holds the status of an individual component of the robot.
# 

# Possible levels of operations
byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte level # level of operation enumerated above 
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status


================================================================================
MSG: diagnostic_msgs/KeyValue
string key # what to label this value when viewing
string value # a value to track over time

================================================================================
MSG: cob_msgs/PowerState
# This message communicates the state of the power system.
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

================================================================================
MSG: cob_msgs/EmergencyStopState
# This message holds the emergency stop (EMStop) status of the robot. It detects wether an EMStop is caused by the safety laserscanner or the emergency stop buttons. Moreover, it gives signalizes wether the EMStop was confirmed (after Button press stop) and the system is free again.

# Possible EMStop States
int16 EMFREE = 0 		# system operatign normal
int16 EMSTOP = 1 		# emergency stop is active (Button pressed; obstacle in safety field of scanner)
int16 EMCONFIRMED = 2 	# emergency stop was confirmed system is reinitializing and going back to normal

bool emergency_button_stop	# true = emergency stop signal is issued by button pressed
bool scanner_stop			# true = emergency stop signal is issued by scanner
int16 emergency_state		# state (including confimation by key-switch), values see above


")



(provide :cob_msgs/DashboardState "ac6926484e7f901e4a07c74a0bf08792")


