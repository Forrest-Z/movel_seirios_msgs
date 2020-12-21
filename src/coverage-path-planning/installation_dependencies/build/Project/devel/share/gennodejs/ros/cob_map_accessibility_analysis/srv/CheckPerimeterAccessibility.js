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

class CheckPerimeterAccessibilityRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center = null;
      this.radius = null;
      this.rotational_sampling_step = null;
    }
    else {
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('rotational_sampling_step')) {
        this.rotational_sampling_step = initObj.rotational_sampling_step
      }
      else {
        this.rotational_sampling_step = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPerimeterAccessibilityRequest
    // Serialize message field [center]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [rotational_sampling_step]
    bufferOffset = _serializer.float64(obj.rotational_sampling_step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPerimeterAccessibilityRequest
    let len;
    let data = new CheckPerimeterAccessibilityRequest(null);
    // Deserialize message field [center]
    data.center = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rotational_sampling_step]
    data.rotational_sampling_step = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_map_accessibility_analysis/CheckPerimeterAccessibilityRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59c2a8fc00319aae0b2c0c3073018e6e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose2D center       # center of the circle whose perimeter should be checked for accessibility, in [m,m,rad]
    float64 radius                    # radius of the circle, in [m]
    float64 rotational_sampling_step  # rotational sampling step width for checking points on the perimeter, in [rad] 
    
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
    const resolved = new CheckPerimeterAccessibilityRequest(null);
    if (msg.center !== undefined) {
      resolved.center = geometry_msgs.msg.Pose2D.Resolve(msg.center)
    }
    else {
      resolved.center = new geometry_msgs.msg.Pose2D()
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.rotational_sampling_step !== undefined) {
      resolved.rotational_sampling_step = msg.rotational_sampling_step;
    }
    else {
      resolved.rotational_sampling_step = 0.0
    }

    return resolved;
    }
};

class CheckPerimeterAccessibilityResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accessible_poses_on_perimeter = null;
    }
    else {
      if (initObj.hasOwnProperty('accessible_poses_on_perimeter')) {
        this.accessible_poses_on_perimeter = initObj.accessible_poses_on_perimeter
      }
      else {
        this.accessible_poses_on_perimeter = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckPerimeterAccessibilityResponse
    // Serialize message field [accessible_poses_on_perimeter]
    // Serialize the length for message field [accessible_poses_on_perimeter]
    bufferOffset = _serializer.uint32(obj.accessible_poses_on_perimeter.length, buffer, bufferOffset);
    obj.accessible_poses_on_perimeter.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose2D.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckPerimeterAccessibilityResponse
    let len;
    let data = new CheckPerimeterAccessibilityResponse(null);
    // Deserialize message field [accessible_poses_on_perimeter]
    // Deserialize array length for message field [accessible_poses_on_perimeter]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.accessible_poses_on_perimeter = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.accessible_poses_on_perimeter[i] = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.accessible_poses_on_perimeter.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cob_map_accessibility_analysis/CheckPerimeterAccessibilityResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f2833ac60e33edc572866f291ef3669';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose2D[] accessible_poses_on_perimeter 	  # array of accessible poses on the perimeter of the circle
    
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
    const resolved = new CheckPerimeterAccessibilityResponse(null);
    if (msg.accessible_poses_on_perimeter !== undefined) {
      resolved.accessible_poses_on_perimeter = new Array(msg.accessible_poses_on_perimeter.length);
      for (let i = 0; i < resolved.accessible_poses_on_perimeter.length; ++i) {
        resolved.accessible_poses_on_perimeter[i] = geometry_msgs.msg.Pose2D.Resolve(msg.accessible_poses_on_perimeter[i]);
      }
    }
    else {
      resolved.accessible_poses_on_perimeter = []
    }

    return resolved;
    }
};

module.exports = {
  Request: CheckPerimeterAccessibilityRequest,
  Response: CheckPerimeterAccessibilityResponse,
  md5sum() { return '26a8e5959c459eafca624877deadd8ec'; },
  datatype() { return 'cob_map_accessibility_analysis/CheckPerimeterAccessibility'; }
};
