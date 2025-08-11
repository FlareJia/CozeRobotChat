// Auto-generated. Do not edit!

// (in-package kuavo_audio_player.srv)


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
      this.music_path = null;
    }
    else {
      if (initObj.hasOwnProperty('music_path')) {
        this.music_path = initObj.music_path
      }
      else {
        this.music_path = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type playmusicRequest
    // Serialize message field [music_path]
    bufferOffset = _serializer.string(obj.music_path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type playmusicRequest
    let len;
    let data = new playmusicRequest(null);
    // Deserialize message field [music_path]
    data.music_path = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.music_path);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kuavo_audio_player/playmusicRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2d09829452828247b2631028a949bf3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string music_path
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new playmusicRequest(null);
    if (msg.music_path !== undefined) {
      resolved.music_path = msg.music_path;
    }
    else {
      resolved.music_path = ''
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
    return 'kuavo_audio_player/playmusicResponse';
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
  md5sum() { return '2b45fccbddec6f75379e3c0de524323f'; },
  datatype() { return 'kuavo_audio_player/playmusic'; }
};
