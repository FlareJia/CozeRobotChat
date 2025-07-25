// Auto-generated. Do not edit!

// (in-package musicplayer_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class playmusicRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.music_number = null;
      this.volume = null;
    }
    else {
      if (initObj.hasOwnProperty('music_number')) {
        this.music_number = initObj.music_number
      }
      else {
        this.music_number = '';
      }
      if (initObj.hasOwnProperty('volume')) {
        this.volume = initObj.volume
      }
      else {
        this.volume = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type playmusicRequest
    // Serialize message field [music_number]
    bufferOffset = _serializer.string(obj.music_number, buffer, bufferOffset);
    // Serialize message field [volume]
    bufferOffset = _serializer.int64(obj.volume, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type playmusicRequest
    let len;
    let data = new playmusicRequest(null);
    // Deserialize message field [music_number]
    data.music_number = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [volume]
    data.volume = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.music_number);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'musicplayer_pkg/playmusicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b4bd61c49344d06e17224a3598f23d8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string music_number
    int64 volume
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new playmusicRequest(null);
    if (msg.music_number !== undefined) {
      resolved.music_number = msg.music_number;
    }
    else {
      resolved.music_number = ''
    }

    if (msg.volume !== undefined) {
      resolved.volume = msg.volume;
    }
    else {
      resolved.volume = 0
    }

    return resolved;
    }
};

class playmusicResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success_flag = null;
    }
    else {
      if (initObj.hasOwnProperty('success_flag')) {
        this.success_flag = initObj.success_flag
      }
      else {
        this.success_flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type playmusicResponse
    // Serialize message field [success_flag]
    bufferOffset = _serializer.bool(obj.success_flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type playmusicResponse
    let len;
    let data = new playmusicResponse(null);
    // Deserialize message field [success_flag]
    data.success_flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'musicplayer_pkg/playmusicResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51d4deda6e3cbea57b8c79590b6cd9bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success_flag
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new playmusicResponse(null);
    if (msg.success_flag !== undefined) {
      resolved.success_flag = msg.success_flag;
    }
    else {
      resolved.success_flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: playmusicRequest,
  Response: playmusicResponse,
  md5sum() { return '3d99283888736c5e18f20ac685e5f8bf'; },
  datatype() { return 'musicplayer_pkg/playmusic'; }
};
