// Auto-generated. Do not edit!

// (in-package cob_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PowerState = require('./PowerState.js');
let EmergencyStopState = require('./EmergencyStopState.js');
let diagnostic_msgs = _finder('diagnostic_msgs');

//-----------------------------------------------------------

class DashboardState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.diagnostics_toplevel_state = null;
      this.power_state = null;
      this.emergency_stop_state = null;
    }
    else {
      if (initObj.hasOwnProperty('diagnostics_toplevel_state')) {
        this.diagnostics_toplevel_state = initObj.diagnostics_toplevel_state
      }
      else {
        this.diagnostics_toplevel_state = new diagnostic_msgs.msg.DiagnosticStatus();
      }
      if (initObj.hasOwnProperty('power_state')) {
        this.power_state = initObj.power_state
      }
      else {
        this.power_state = new PowerState();
      }
      if (initObj.hasOwnProperty('emergency_stop_state')) {
        this.emergency_stop_state = initObj.emergency_stop_state
      }
      else {
        this.emergency_stop_state = new EmergencyStopState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DashboardState
    // Serialize message field [diagnostics_toplevel_state]
    bufferOffset = diagnostic_msgs.msg.DiagnosticStatus.serialize(obj.diagnostics_toplevel_state, buffer, bufferOffset);
    // Serialize message field [power_state]
    bufferOffset = PowerState.serialize(obj.power_state, buffer, bufferOffset);
    // Serialize message field [emergency_stop_state]
    bufferOffset = EmergencyStopState.serialize(obj.emergency_stop_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DashboardState
    let len;
    let data = new DashboardState(null);
    // Deserialize message field [diagnostics_toplevel_state]
    data.diagnostics_toplevel_state = diagnostic_msgs.msg.DiagnosticStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [power_state]
    data.power_state = PowerState.deserialize(buffer, bufferOffset);
    // Deserialize message field [emergency_stop_state]
    data.emergency_stop_state = EmergencyStopState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += diagnostic_msgs.msg.DiagnosticStatus.getMessageSize(object.diagnostics_toplevel_state);
    length += PowerState.getMessageSize(object.power_state);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cob_msgs/DashboardState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac6926484e7f901e4a07c74a0bf08792';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This message communicates state information that might be used by a dashboard application.
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
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DashboardState(null);
    if (msg.diagnostics_toplevel_state !== undefined) {
      resolved.diagnostics_toplevel_state = diagnostic_msgs.msg.DiagnosticStatus.Resolve(msg.diagnostics_toplevel_state)
    }
    else {
      resolved.diagnostics_toplevel_state = new diagnostic_msgs.msg.DiagnosticStatus()
    }

    if (msg.power_state !== undefined) {
      resolved.power_state = PowerState.Resolve(msg.power_state)
    }
    else {
      resolved.power_state = new PowerState()
    }

    if (msg.emergency_stop_state !== undefined) {
      resolved.emergency_stop_state = EmergencyStopState.Resolve(msg.emergency_stop_state)
    }
    else {
      resolved.emergency_stop_state = new EmergencyStopState()
    }

    return resolved;
    }
};

module.exports = DashboardState;
