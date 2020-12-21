// Auto-generated. Do not edit!

// (in-package cob_map_accessibility_analysis.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class CheckPointAccessibilityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.points_to_check = null;
      this.approach_path_accessibility_check = null;
    }
    else {
      if (initObj.hasOwnProperty('points_to_check')) {
        this.points_to_check = initObj.points_to_check
      }
      else {
        this.points_to_check = [];
      }
      if (initObj.hasOwnProperty('approach_path_accessibility_check')) {
        this.approach_path_accessibility_check = initObj.approach_path_accessibility_check
      }
      else {
        this.approach_path_accessibility_check = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPointAccessibilityRequest
    // Serialize message field [points_to_check]
    // Serialize the length for message field [points_to_check]
    bufferOffset = _serializer.uint32(obj.points_to_check.length, buffer, bufferOffset);
    obj.points_to_check.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose2D.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [approach_path_accessibility_check]
    bufferOffset = _serializer.bool(obj.approach_path_accessibility_check, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPointAccessibilityRequest
    let len;
    let data = new CheckPointAccessibilityRequest(null);
    // Deserialize message field [points_to_check]
    // Deserialize array length for message field [points_to_check]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.points_to_check = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.points_to_check[i] = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [approach_path_accessibility_check]
    data.approach_path_accessibility_check = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.points_to_check.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_map_accessibility_analysis/CheckPointAccessibilityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b406244933de86ab08575478d23717b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose2D[] points_to_check    # array of points which should be checked for accessibility
    bool approach_path_accessibility_check    # if true, the path to a goal position must be accessible as well
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPointAccessibilityRequest(null);
    if (msg.points_to_check !== undefined) {
      resolved.points_to_check = new Array(msg.points_to_check.length);
      for (let i = 0; i < resolved.points_to_check.length; ++i) {
        resolved.points_to_check[i] = geometry_msgs.msg.Pose2D.Resolve(msg.points_to_check[i]);
      }
    }
    else {
      resolved.points_to_check = []
    }

    if (msg.approach_path_accessibility_check !== undefined) {
      resolved.approach_path_accessibility_check = msg.approach_path_accessibility_check;
    }
    else {
      resolved.approach_path_accessibility_check = false
    }

    return resolved;
    }
};

class CheckPointAccessibilityResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accessibility_flags = null;
    }
    else {
      if (initObj.hasOwnProperty('accessibility_flags')) {
        this.accessibility_flags = initObj.accessibility_flags
      }
      else {
        this.accessibility_flags = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPointAccessibilityResponse
    // Serialize message field [accessibility_flags]
    bufferOffset = _arraySerializer.bool(obj.accessibility_flags, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPointAccessibilityResponse
    let len;
    let data = new CheckPointAccessibilityResponse(null);
    // Deserialize message field [accessibility_flags]
    data.accessibility_flags = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.accessibility_flags.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_map_accessibility_analysis/CheckPointAccessibilityResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f3c377d86bd78d373d82f6a042c05ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool[] accessibility_flags    			  # array of booleans which correspond to the points and define accessibility (true=free, false=obstacle)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckPointAccessibilityResponse(null);
    if (msg.accessibility_flags !== undefined) {
      resolved.accessibility_flags = msg.accessibility_flags;
    }
    else {
      resolved.accessibility_flags = []
    }

    return resolved;
    }
};

module.exports = {
  Request: CheckPointAccessibilityRequest,
  Response: CheckPointAccessibilityResponse,
  md5sum() { return 'c42c7449dffc73e50011cd7a1eb83e23'; },
  datatype() { return 'cob_map_accessibility_analysis/CheckPointAccessibility'; }
};
