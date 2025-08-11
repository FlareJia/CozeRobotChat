; Auto-generated. Do not edit!


(cl:in-package file_transfer-srv)


;//! \htmlinclude FileTransfer-request.msg.html

(cl:defclass <FileTransfer-request> (roslisp-msg-protocol:ros-message)
  ((source_path
    :reader source_path
    :initarg :source_path
    :type cl:string
    :initform "")
   (target_path
    :reader target_path
    :initarg :target_path
    :type cl:string
    :initform "")
   (is_download
    :reader is_download
    :initarg :is_download
    :type cl:boolean
    :initform cl:nil)
   (file_data
    :reader file_data
    :initarg :file_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (file_size
    :reader file_size
    :initarg :file_size
    :type cl:integer
    :initform 0))
)

(cl:defclass FileTransfer-request (<FileTransfer-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FileTransfer-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FileTransfer-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name file_transfer-srv:<FileTransfer-request> is deprecated: use file_transfer-srv:FileTransfer-request instead.")))

(cl:ensure-generic-function 'source_path-val :lambda-list '(m))
(cl:defmethod source_path-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:source_path-val is deprecated.  Use file_transfer-srv:source_path instead.")
  (source_path m))

(cl:ensure-generic-function 'target_path-val :lambda-list '(m))
(cl:defmethod target_path-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:target_path-val is deprecated.  Use file_transfer-srv:target_path instead.")
  (target_path m))

(cl:ensure-generic-function 'is_download-val :lambda-list '(m))
(cl:defmethod is_download-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:is_download-val is deprecated.  Use file_transfer-srv:is_download instead.")
  (is_download m))

(cl:ensure-generic-function 'file_data-val :lambda-list '(m))
(cl:defmethod file_data-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:file_data-val is deprecated.  Use file_transfer-srv:file_data instead.")
  (file_data m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:success-val is deprecated.  Use file_transfer-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:message-val is deprecated.  Use file_transfer-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'file_size-val :lambda-list '(m))
(cl:defmethod file_size-val ((m <FileTransfer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_transfer-srv:file_size-val is deprecated.  Use file_transfer-srv:file_size instead.")
  (file_size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FileTransfer-request>) ostream)
  "Serializes a message object of type '<FileTransfer-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source_path))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target_path))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_download) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'file_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'file_data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let* ((signed (cl:slot-value msg 'file_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FileTransfer-request>) istream)
  "Deserializes a message object of type '<FileTransfer-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'is_download) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'file_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'file_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file_size) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FileTransfer-request>)))
  "Returns string type for a service object of type '<FileTransfer-request>"
  "file_transfer/FileTransferRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FileTransfer-request)))
  "Returns string type for a service object of type 'FileTransfer-request"
  "file_transfer/FileTransferRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FileTransfer-request>)))
  "Returns md5sum for a message object of type '<FileTransfer-request>"
  "1f6b3d4d49e3b3db7a6ad1ca13631528")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FileTransfer-request)))
  "Returns md5sum for a message object of type 'FileTransfer-request"
  "1f6b3d4d49e3b3db7a6ad1ca13631528")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FileTransfer-request>)))
  "Returns full string definition for message of type '<FileTransfer-request>"
  (cl:format cl:nil "# Request~%string source_path    # 源文件路径(调用方指定)~%string target_path    # 目标文件路径(服务端指定)~%bool is_download      # true=下位机->上位机(下载), false=上位机->下位机(上传)~%uint8[] file_data     # 文件数据(二进制)~%~%# Response~%bool success~%string message~%int64 file_size~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FileTransfer-request)))
  "Returns full string definition for message of type 'FileTransfer-request"
  (cl:format cl:nil "# Request~%string source_path    # 源文件路径(调用方指定)~%string target_path    # 目标文件路径(服务端指定)~%bool is_download      # true=下位机->上位机(下载), false=上位机->下位机(上传)~%uint8[] file_data     # 文件数据(二进制)~%~%# Response~%bool success~%string message~%int64 file_size~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FileTransfer-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'source_path))
     4 (cl:length (cl:slot-value msg 'target_path))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'file_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     4 (cl:length (cl:slot-value msg 'message))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FileTransfer-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FileTransfer-request
    (cl:cons ':source_path (source_path msg))
    (cl:cons ':target_path (target_path msg))
    (cl:cons ':is_download (is_download msg))
    (cl:cons ':file_data (file_data msg))
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':file_size (file_size msg))
))
;//! \htmlinclude FileTransfer-response.msg.html

(cl:defclass <FileTransfer-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FileTransfer-response (<FileTransfer-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FileTransfer-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FileTransfer-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name file_transfer-srv:<FileTransfer-response> is deprecated: use file_transfer-srv:FileTransfer-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FileTransfer-response>) ostream)
  "Serializes a message object of type '<FileTransfer-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FileTransfer-response>) istream)
  "Deserializes a message object of type '<FileTransfer-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FileTransfer-response>)))
  "Returns string type for a service object of type '<FileTransfer-response>"
  "file_transfer/FileTransferResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FileTransfer-response)))
  "Returns string type for a service object of type 'FileTransfer-response"
  "file_transfer/FileTransferResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FileTransfer-response>)))
  "Returns md5sum for a message object of type '<FileTransfer-response>"
  "1f6b3d4d49e3b3db7a6ad1ca13631528")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FileTransfer-response)))
  "Returns md5sum for a message object of type 'FileTransfer-response"
  "1f6b3d4d49e3b3db7a6ad1ca13631528")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FileTransfer-response>)))
  "Returns full string definition for message of type '<FileTransfer-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FileTransfer-response)))
  "Returns full string definition for message of type 'FileTransfer-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FileTransfer-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FileTransfer-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FileTransfer-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FileTransfer)))
  'FileTransfer-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FileTransfer)))
  'FileTransfer-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FileTransfer)))
  "Returns string type for a service object of type '<FileTransfer>"
  "file_transfer/FileTransfer")