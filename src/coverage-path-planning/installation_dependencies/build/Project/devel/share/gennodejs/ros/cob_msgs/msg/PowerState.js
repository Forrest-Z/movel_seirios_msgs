// Auto-generated. Do not edit!

// (in-package cob_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PowerState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.voltage = null;
      this.current = null;
      this.power_consumption = null;
      this.remaining_capacity = null;
      this.relative_remaining_capacity = null;
      this.charging = null;
      this.time_remaining = null;
      this.temperature = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('power_consumption')) {
        this.power_consumption = initObj.power_consumption
      }
      else {
        this.power_consumption = 0.0;
      }
      if (initObj.hasOwnProperty('remaining_capacity')) {
        this.remaining_capacity = initObj.remaining_capacity
      }
      else {
        this.remaining_capacity = 0.0;
      }
      if (initObj.hasOwnProperty('relative_remaining_capacity')) {
        this.relative_remaining_capacity = initObj.relative_remaining_capacity
      }
      else {
        this.relative_remaining_capacity = 0.0;
      }
      if (initObj.hasOwnProperty('charging')) {
        this.charging = initObj.charging
      }
      else {
        this.charging = false;
      }
      if (initObj.hasOwnProperty('time_remaining')) {
        this.time_remaining = initObj.time_remaining
      }
      else {
        this.time_remaining = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PowerState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float64(obj.voltage, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float64(obj.current, buffer, bufferOffset);
    // Serialize message field [power_consumption]
    bufferOffset = _serializer.float64(obj.power_consumption, buffer, bufferOffset);
    // Serialize message field [remaining_capacity]
    bufferOffset = _serializer.float64(obj.remaining_capacity, buffer, bufferOffset);
    // Serialize message field [relative_remaining_capacity]
    bufferOffset = _serializer.float64(obj.relative_remaining_capacity, buffer, bufferOffset);
    // Serialize message field [charging]
    bufferOffset = _serializer.bool(obj.charging, buffer, bufferOffset);
    // Serialize message field [time_remaining]
    bufferOffset = _serializer.float64(obj.time_remaining, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.float64(obj.temperature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PowerState
    let len;
    let data = new PowerState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [power_consumption]
    data.power_consumption = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [remaining_capacity]
    data.remaining_capacity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [relative_remaining_capacity]
    data.relative_remaining_capacity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [charging]
    data.charging = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [time_remaining]
    data.time_remaining = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cob_msgs/PowerState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '70361e18e36187b93dab6d6ec8ca15e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PowerState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.power_consumption !== undefined) {
      resolved.power_consumption = msg.power_consumption;
    }
    else {
      resolved.power_consumption = 0.0
    }

    if (msg.remaining_capacity !== undefined) {
      resolved.remaining_capacity = msg.remaining_capacity;
    }
    else {
      resolved.remaining_capacity = 0.0
    }

    if (msg.relative_remaining_capacity !== undefined) {
      resolved.relative_remaining_capacity = msg.relative_remaining_capacity;
    }
    else {
      resolved.relative_remaining_capacity = 0.0
    }

    if (msg.charging !== undefined) {
      resolved.charging = msg.charging;
    }
    else {
      resolved.charging = false
    }

    if (msg.time_remaining !== undefined) {
      resolved.time_remaining = msg.time_remaining;
    }
    else {
      resolved.time_remaining = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
    }

    return resolved;
    }
};

module.exports = PowerState;
