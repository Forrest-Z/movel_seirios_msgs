// Auto-generated. Do not edit!

// (in-package cob_phidgets.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetTriggerValueRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.trigger_value = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('trigger_value')) {
        this.trigger_value = initObj.trigger_value
      }
      else {
        this.trigger_value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTriggerValueRequest
    // Serialize message field [index]
    bufferOffset = _serializer.int8(obj.index, buffer, bufferOffset);
    // Serialize message field [trigger_value]
    bufferOffset = _serializer.uint16(obj.trigger_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTriggerValueRequest
    let len;
    let data = new SetTriggerValueRequest(null);
    // Deserialize message field [index]
    data.index = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [trigger_value]
    data.trigger_value = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_phidgets/SetTriggerValueRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f5eabd2487a9b62214c60cbc4be257ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 index
    uint16 trigger_value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTriggerValueRequest(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.trigger_value !== undefined) {
      resolved.trigger_value = msg.trigger_value;
    }
    else {
      resolved.trigger_value = 0
    }

    return resolved;
    }
};

class SetTriggerValueResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTriggerValueResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTriggerValueResponse
    let len;
    let data = new SetTriggerValueResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_phidgets/SetTriggerValueResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTriggerValueResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetTriggerValueRequest,
  Response: SetTriggerValueResponse,
  md5sum() { return 'f5eabd2487a9b62214c60cbc4be257ee'; },
  datatype() { return 'cob_phidgets/SetTriggerValue'; }
};
