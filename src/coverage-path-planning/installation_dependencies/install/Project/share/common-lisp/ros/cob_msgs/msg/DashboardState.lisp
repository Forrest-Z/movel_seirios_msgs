; Auto-generated. Do not edit!


(cl:in-package cob_msgs-msg)


;//! \htmlinclude DashboardState.msg.html

(cl:defclass <DashboardState> (roslisp-msg-protocol:ros-message)
  ((diagnostics_toplevel_state
    :reader diagnostics_toplevel_state
    :initarg :diagnostics_toplevel_state
    :type diagnostic_msgs-msg:DiagnosticStatus
    :initform (cl:make-instance 'diagnostic_msgs-msg:DiagnosticStatus))
   (power_state
    :reader power_state
    :initarg :power_state
    :type cob_msgs-msg:PowerState
    :initform (cl:make-instance 'cob_msgs-msg:PowerState))
   (emergency_stop_state
    :reader emergency_stop_state
    :initarg :emergency_stop_state
    :type cob_msgs-msg:EmergencyStopState
    :initform (cl:make-instance 'cob_msgs-msg:EmergencyStopState)))
)

(cl:defclass DashboardState (<DashboardState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DashboardState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DashboardState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cob_msgs-msg:<DashboardState> is deprecated: use cob_msgs-msg:DashboardState instead.")))

(cl:ensure-generic-function 'diagnostics_toplevel_state-val :lambda-list '(m))
(cl:defmethod diagnostics_toplevel_state-val ((m <DashboardState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:diagnostics_toplevel_state-val is deprecated.  Use cob_msgs-msg:diagnostics_toplevel_state instead.")
  (diagnostics_toplevel_state m))

(cl:ensure-generic-function 'power_state-val :lambda-list '(m))
(cl:defmethod power_state-val ((m <DashboardState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:power_state-val is deprecated.  Use cob_msgs-msg:power_state instead.")
  (power_state m))

(cl:ensure-generic-function 'emergency_stop_state-val :lambda-list '(m))
(cl:defmethod emergency_stop_state-val ((m <DashboardState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cob_msgs-msg:emergency_stop_state-val is deprecated.  Use cob_msgs-msg:emergency_stop_state instead.")
  (emergency_stop_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DashboardState>) ostream)
  "Serializes a message object of type '<DashboardState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'diagnostics_toplevel_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'power_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'emergency_stop_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DashboardState>) istream)
  "Deserializes a message object of type '<DashboardState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'diagnostics_toplevel_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'power_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'emergency_stop_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DashboardState>)))
  "Returns string type for a message object of type '<DashboardState>"
  "cob_msgs/DashboardState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DashboardState)))
  "Returns string type for a message object of type 'DashboardState"
  "cob_msgs/DashboardState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DashboardState>)))
  "Returns md5sum for a message object of type '<DashboardState>"
  "ac6926484e7f901e4a07c74a0bf08792")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DashboardState)))
  "Returns md5sum for a message object of type 'DashboardState"
  "ac6926484e7f901e4a07c74a0bf08792")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DashboardState>)))
  "Returns full string definition for message of type '<DashboardState>"
  (cl:format cl:nil "# This message communicates state information that might be used by a dashboard application.~%diagnostic_msgs/DiagnosticStatus diagnostics_toplevel_state~%~%cob_msgs/PowerState power_state~%~%cob_msgs/EmergencyStopState emergency_stop_state~%~%================================================================================~%MSG: diagnostic_msgs/DiagnosticStatus~%# This message holds the status of an individual component of the robot.~%# ~%~%# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3~%~%byte level # level of operation enumerated above ~%string name # a description of the test/component reporting~%string message # a description of the status~%string hardware_id # a hardware unique string~%KeyValue[] values # an array of values associated with the status~%~%~%================================================================================~%MSG: diagnostic_msgs/KeyValue~%string key # what to label this value when viewing~%string value # a value to track over time~%~%================================================================================~%MSG: cob_msgs/PowerState~%# This message communicates the state of the power system.~%Header header~%float64 voltage                     # [V]~%float64 current                     # [A]~%float64 power_consumption           # [W] can only be calculated if not charging~%float64 remaining_capacity          # [Ah]~%float64 relative_remaining_capacity # [0..100] percent of maximum capacity (parameter max_capacity)~%bool charging                       # flag if robot is connected to external power or not~%float64 time_remaining              # [h] estimated time to empty or fully charged~%float64 temperature                 # [Celsius] temperature of the battery~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: cob_msgs/EmergencyStopState~%# This message holds the emergency stop (EMStop) status of the robot. It detects wether an EMStop is caused by the safety laserscanner or the emergency stop buttons. Moreover, it gives signalizes wether the EMStop was confirmed (after Button press stop) and the system is free again.~%~%# Possible EMStop States~%int16 EMFREE = 0 		# system operatign normal~%int16 EMSTOP = 1 		# emergency stop is active (Button pressed; obstacle in safety field of scanner)~%int16 EMCONFIRMED = 2 	# emergency stop was confirmed system is reinitializing and going back to normal~%~%bool emergency_button_stop	# true = emergency stop signal is issued by button pressed~%bool scanner_stop			# true = emergency stop signal is issued by scanner~%int16 emergency_state		# state (including confimation by key-switch), values see above~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DashboardState)))
  "Returns full string definition for message of type 'DashboardState"
  (cl:format cl:nil "# This message communicates state information that might be used by a dashboard application.~%diagnostic_msgs/DiagnosticStatus diagnostics_toplevel_state~%~%cob_msgs/PowerState power_state~%~%cob_msgs/EmergencyStopState emergency_stop_state~%~%================================================================================~%MSG: diagnostic_msgs/DiagnosticStatus~%# This message holds the status of an individual component of the robot.~%# ~%~%# Possible levels of operations~%byte OK=0~%byte WARN=1~%byte ERROR=2~%byte STALE=3~%~%byte level # level of operation enumerated above ~%string name # a description of the test/component reporting~%string message # a description of the status~%string hardware_id # a hardware unique string~%KeyValue[] values # an array of values associated with the status~%~%~%================================================================================~%MSG: diagnostic_msgs/KeyValue~%string key # what to label this value when viewing~%string value # a value to track over time~%~%================================================================================~%MSG: cob_msgs/PowerState~%# This message communicates the state of the power system.~%Header header~%float64 voltage                     # [V]~%float64 current                     # [A]~%float64 power_consumption           # [W] can only be calculated if not charging~%float64 remaining_capacity          # [Ah]~%float64 relative_remaining_capacity # [0..100] percent of maximum capacity (parameter max_capacity)~%bool charging                       # flag if robot is connected to external power or not~%float64 time_remaining              # [h] estimated time to empty or fully charged~%float64 temperature                 # [Celsius] temperature of the battery~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: cob_msgs/EmergencyStopState~%# This message holds the emergency stop (EMStop) status of the robot. It detects wether an EMStop is caused by the safety laserscanner or the emergency stop buttons. Moreover, it gives signalizes wether the EMStop was confirmed (after Button press stop) and the system is free again.~%~%# Possible EMStop States~%int16 EMFREE = 0 		# system operatign normal~%int16 EMSTOP = 1 		# emergency stop is active (Button pressed; obstacle in safety field of scanner)~%int16 EMCONFIRMED = 2 	# emergency stop was confirmed system is reinitializing and going back to normal~%~%bool emergency_button_stop	# true = emergency stop signal is issued by button pressed~%bool scanner_stop			# true = emergency stop signal is issued by scanner~%int16 emergency_state		# state (including confimation by key-switch), values see above~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DashboardState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'diagnostics_toplevel_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'power_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'emergency_stop_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DashboardState>))
  "Converts a ROS message object to a list"
  (cl:list 'DashboardState
    (cl:cons ':diagnostics_toplevel_state (diagnostics_toplevel_state msg))
    (cl:cons ':power_state (power_state msg))
    (cl:cons ':emergency_stop_state (emergency_stop_state msg))
))
