// Auto-generated. Do not edit!

// (in-package cob_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Network = require('./Network.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SiteSurvey {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.networks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('networks')) {
        this.networks = initObj.networks
      }
      else {
        this.networks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SiteSurvey
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [networks]
    // Serialize the length for message field [networks]
    bufferOffset = _serializer.uint32(obj.networks.length, buffer, bufferOffset);
    obj.networks.forEach((val) => {
      bufferOffset = Network.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SiteSurvey
    let len;
    let data = new SiteSurvey(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [networks]
    // Deserialize array length for message field [networks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.networks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.networks[i] = Network.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.networks.forEach((val) => {
      length += Network.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cob_msgs/SiteSurvey';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1063b16bb121ef190ae5edfe09d94ec';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SiteSurvey(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.networks !== undefined) {
      resolved.networks = new Array(msg.networks.length);
      for (let i = 0; i < resolved.networks.length; ++i) {
        resolved.networks[i] = Network.Resolve(msg.networks[i]);
      }
    }
    else {
      resolved.networks = []
    }

    return resolved;
    }
};

module.exports = SiteSurvey;
