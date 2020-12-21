// Auto-generated. Do not edit!

// (in-package cob_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Network {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.macattr = null;
      this.essid = null;
      this.channel = null;
      this.rssi = null;
      this.noise = null;
      this.beacon = null;
    }
    else {
      if (initObj.hasOwnProperty('macattr')) {
        this.macattr = initObj.macattr
      }
      else {
        this.macattr = '';
      }
      if (initObj.hasOwnProperty('essid')) {
        this.essid = initObj.essid
      }
      else {
        this.essid = '';
      }
      if (initObj.hasOwnProperty('channel')) {
        this.channel = initObj.channel
      }
      else {
        this.channel = 0;
      }
      if (initObj.hasOwnProperty('rssi')) {
        this.rssi = initObj.rssi
      }
      else {
        this.rssi = 0;
      }
      if (initObj.hasOwnProperty('noise')) {
        this.noise = initObj.noise
      }
      else {
        this.noise = 0;
      }
      if (initObj.hasOwnProperty('beacon')) {
        this.beacon = initObj.beacon
      }
      else {
        this.beacon = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Network
    // Serialize message field [macattr]
    bufferOffset = _serializer.string(obj.macattr, buffer, bufferOffset);
    // Serialize message field [essid]
    bufferOffset = _serializer.string(obj.essid, buffer, bufferOffset);
    // Serialize message field [channel]
    bufferOffset = _serializer.int32(obj.channel, buffer, bufferOffset);
    // Serialize message field [rssi]
    bufferOffset = _serializer.int32(obj.rssi, buffer, bufferOffset);
    // Serialize message field [noise]
    bufferOffset = _serializer.int32(obj.noise, buffer, bufferOffset);
    // Serialize message field [beacon]
    bufferOffset = _serializer.int32(obj.beacon, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Network
    let len;
    let data = new Network(null);
    // Deserialize message field [macattr]
    data.macattr = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [essid]
    data.essid = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [channel]
    data.channel = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [rssi]
    data.rssi = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [noise]
    data.noise = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [beacon]
    data.beacon = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.macattr);
    length += _getByteLength(object.essid);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cob_msgs/Network';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0854419660dc197dd94305843bee07f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Network(null);
    if (msg.macattr !== undefined) {
      resolved.macattr = msg.macattr;
    }
    else {
      resolved.macattr = ''
    }

    if (msg.essid !== undefined) {
      resolved.essid = msg.essid;
    }
    else {
      resolved.essid = ''
    }

    if (msg.channel !== undefined) {
      resolved.channel = msg.channel;
    }
    else {
      resolved.channel = 0
    }

    if (msg.rssi !== undefined) {
      resolved.rssi = msg.rssi;
    }
    else {
      resolved.rssi = 0
    }

    if (msg.noise !== undefined) {
      resolved.noise = msg.noise;
    }
    else {
      resolved.noise = 0
    }

    if (msg.beacon !== undefined) {
      resolved.beacon = msg.beacon;
    }
    else {
      resolved.beacon = 0
    }

    return resolved;
    }
};

module.exports = Network;
