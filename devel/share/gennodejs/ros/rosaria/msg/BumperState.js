// Auto-generated. Do not edit!

// (in-package rosaria.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BumperState {
  constructor() {
    this.header = new std_msgs.msg.Header();
    this.front_bumpers = [];
    this.rear_bumpers = [];
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type BumperState
    // Serialize message field [header]
    bufferInfo = std_msgs.msg.Header.serialize(obj.header, bufferInfo);
    // Serialize the length for message field [front_bumpers]
    bufferInfo = _serializer.uint32(obj.front_bumpers.length, bufferInfo);
    // Serialize message field [front_bumpers]
    obj.front_bumpers.forEach((val) => {
      bufferInfo = _serializer.bool(val, bufferInfo);
    });
    // Serialize the length for message field [rear_bumpers]
    bufferInfo = _serializer.uint32(obj.rear_bumpers.length, bufferInfo);
    // Serialize message field [rear_bumpers]
    obj.rear_bumpers.forEach((val) => {
      bufferInfo = _serializer.bool(val, bufferInfo);
    });
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type BumperState
    let tmp;
    let len;
    let data = new BumperState();
    // Deserialize message field [header]
    tmp = std_msgs.msg.Header.deserialize(buffer);
    data.header = tmp.data;
    buffer = tmp.buffer;
    // Deserialize array length for message field [front_bumpers]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [front_bumpers]
    data.front_bumpers = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.bool(buffer);
      data.front_bumpers[i] = tmp.data;
      buffer = tmp.buffer;
    }
    // Deserialize array length for message field [rear_bumpers]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [rear_bumpers]
    data.rear_bumpers = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = _deserializer.bool(buffer);
      data.rear_bumpers[i] = tmp.data;
      buffer = tmp.buffer;
    }
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosaria/BumperState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f81947761ff7e166a3bbaf937b9869b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    bool[] front_bumpers
    bool[] rear_bumpers
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

};

module.exports = BumperState;
