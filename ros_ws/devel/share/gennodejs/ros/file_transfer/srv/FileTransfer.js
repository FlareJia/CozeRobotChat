// Auto-generated. Do not edit!

// (in-package file_transfer.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class FileTransferRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source_path = null;
      this.target_path = null;
      this.is_download = null;
      this.file_data = null;
      this.success = null;
      this.message = null;
      this.file_size = null;
    }
    else {
      if (initObj.hasOwnProperty('source_path')) {
        this.source_path = initObj.source_path
      }
      else {
        this.source_path = '';
      }
      if (initObj.hasOwnProperty('target_path')) {
        this.target_path = initObj.target_path
      }
      else {
        this.target_path = '';
      }
      if (initObj.hasOwnProperty('is_download')) {
        this.is_download = initObj.is_download
      }
      else {
        this.is_download = false;
      }
      if (initObj.hasOwnProperty('file_data')) {
        this.file_data = initObj.file_data
      }
      else {
        this.file_data = [];
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('file_size')) {
        this.file_size = initObj.file_size
      }
      else {
        this.file_size = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FileTransferRequest
    // Serialize message field [source_path]
    bufferOffset = _serializer.string(obj.source_path, buffer, bufferOffset);
    // Serialize message field [target_path]
    bufferOffset = _serializer.string(obj.target_path, buffer, bufferOffset);
    // Serialize message field [is_download]
    bufferOffset = _serializer.bool(obj.is_download, buffer, bufferOffset);
    // Serialize message field [file_data]
    bufferOffset = _arraySerializer.uint8(obj.file_data, buffer, bufferOffset, null);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [file_size]
    bufferOffset = _serializer.int64(obj.file_size, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FileTransferRequest
    let len;
    let data = new FileTransferRequest(null);
    // Deserialize message field [source_path]
    data.source_path = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_path]
    data.target_path = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_download]
    data.is_download = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [file_data]
    data.file_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [file_size]
    data.file_size = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.source_path);
    length += _getByteLength(object.target_path);
    length += object.file_data.length;
    length += _getByteLength(object.message);
    return length + 26;
  }

  static datatype() {
    // Returns string type for a service object
    return 'file_transfer/FileTransferRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f6b3d4d49e3b3db7a6ad1ca13631528';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    string source_path    # 源文件路径(调用方指定)
    string target_path    # 目标文件路径(服务端指定)
    bool is_download      # true=下位机->上位机(下载), false=上位机->下位机(上传)
    uint8[] file_data     # 文件数据(二进制)
    
    # Response
    bool success
    string message
    int64 file_size
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FileTransferRequest(null);
    if (msg.source_path !== undefined) {
      resolved.source_path = msg.source_path;
    }
    else {
      resolved.source_path = ''
    }

    if (msg.target_path !== undefined) {
      resolved.target_path = msg.target_path;
    }
    else {
      resolved.target_path = ''
    }

    if (msg.is_download !== undefined) {
      resolved.is_download = msg.is_download;
    }
    else {
      resolved.is_download = false
    }

    if (msg.file_data !== undefined) {
      resolved.file_data = msg.file_data;
    }
    else {
      resolved.file_data = []
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.file_size !== undefined) {
      resolved.file_size = msg.file_size;
    }
    else {
      resolved.file_size = 0
    }

    return resolved;
    }
};

class FileTransferResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FileTransferResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FileTransferResponse
    let len;
    let data = new FileTransferResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'file_transfer/FileTransferResponse';
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
    const resolved = new FileTransferResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: FileTransferRequest,
  Response: FileTransferResponse,
  md5sum() { return '1f6b3d4d49e3b3db7a6ad1ca13631528'; },
  datatype() { return 'file_transfer/FileTransfer'; }
};
