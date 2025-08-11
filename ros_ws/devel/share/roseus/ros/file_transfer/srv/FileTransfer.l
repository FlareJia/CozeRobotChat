;; Auto-generated. Do not edit!


(when (boundp 'file_transfer::FileTransfer)
  (if (not (find-package "FILE_TRANSFER"))
    (make-package "FILE_TRANSFER"))
  (shadow 'FileTransfer (find-package "FILE_TRANSFER")))
(unless (find-package "FILE_TRANSFER::FILETRANSFER")
  (make-package "FILE_TRANSFER::FILETRANSFER"))
(unless (find-package "FILE_TRANSFER::FILETRANSFERREQUEST")
  (make-package "FILE_TRANSFER::FILETRANSFERREQUEST"))
(unless (find-package "FILE_TRANSFER::FILETRANSFERRESPONSE")
  (make-package "FILE_TRANSFER::FILETRANSFERRESPONSE"))

(in-package "ROS")





(defclass file_transfer::FileTransferRequest
  :super ros::object
  :slots (_source_path _target_path _is_download _file_data _success _message _file_size ))

(defmethod file_transfer::FileTransferRequest
  (:init
   (&key
    ((:source_path __source_path) "")
    ((:target_path __target_path) "")
    ((:is_download __is_download) nil)
    ((:file_data __file_data) (make-array 0 :initial-element 0 :element-type :char))
    ((:success __success) nil)
    ((:message __message) "")
    ((:file_size __file_size) 0)
    )
   (send-super :init)
   (setq _source_path (string __source_path))
   (setq _target_path (string __target_path))
   (setq _is_download __is_download)
   (setq _file_data __file_data)
   (setq _success __success)
   (setq _message (string __message))
   (setq _file_size (round __file_size))
   self)
  (:source_path
   (&optional __source_path)
   (if __source_path (setq _source_path __source_path)) _source_path)
  (:target_path
   (&optional __target_path)
   (if __target_path (setq _target_path __target_path)) _target_path)
  (:is_download
   (&optional (__is_download :null))
   (if (not (eq __is_download :null)) (setq _is_download __is_download)) _is_download)
  (:file_data
   (&optional __file_data)
   (if __file_data (setq _file_data __file_data)) _file_data)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:file_size
   (&optional __file_size)
   (if __file_size (setq _file_size __file_size)) _file_size)
  (:serialization-length
   ()
   (+
    ;; string _source_path
    4 (length _source_path)
    ;; string _target_path
    4 (length _target_path)
    ;; bool _is_download
    1
    ;; uint8[] _file_data
    (* 1    (length _file_data)) 4
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; int64 _file_size
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _source_path
       (write-long (length _source_path) s) (princ _source_path s)
     ;; string _target_path
       (write-long (length _target_path) s) (princ _target_path s)
     ;; bool _is_download
       (if _is_download (write-byte -1 s) (write-byte 0 s))
     ;; uint8[] _file_data
     (write-long (length _file_data) s)
     (princ _file_data s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; int64 _file_size
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _file_size (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _file_size) (= (length (_file_size . bv)) 2)) ;; bignum
              (write-long (ash (elt (_file_size . bv) 0) 0) s)
              (write-long (ash (elt (_file_size . bv) 1) -1) s))
             ((and (class _file_size) (= (length (_file_size . bv)) 1)) ;; big1
              (write-long (elt (_file_size . bv) 0) s)
              (write-long (if (>= _file_size 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _file_size s)(write-long (if (>= _file_size 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _source_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _target_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_download
     (setq _is_download (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8[] _file_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _file_data (make-array n :element-type :char))
     (replace _file_data buf :start2 ptr-) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _file_size
#+(or :alpha :irix6 :x86_64)
      (setf _file_size (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _file_size (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass file_transfer::FileTransferResponse
  :super ros::object
  :slots ())

(defmethod file_transfer::FileTransferResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass file_transfer::FileTransfer
  :super ros::object
  :slots ())

(setf (get file_transfer::FileTransfer :md5sum-) "1f6b3d4d49e3b3db7a6ad1ca13631528")
(setf (get file_transfer::FileTransfer :datatype-) "file_transfer/FileTransfer")
(setf (get file_transfer::FileTransfer :request) file_transfer::FileTransferRequest)
(setf (get file_transfer::FileTransfer :response) file_transfer::FileTransferResponse)

(defmethod file_transfer::FileTransferRequest
  (:response () (instance file_transfer::FileTransferResponse :init)))

(setf (get file_transfer::FileTransferRequest :md5sum-) "1f6b3d4d49e3b3db7a6ad1ca13631528")
(setf (get file_transfer::FileTransferRequest :datatype-) "file_transfer/FileTransferRequest")
(setf (get file_transfer::FileTransferRequest :definition-)
      "# Request
string source_path    # 源文件路径(调用方指定)
string target_path    # 目标文件路径(服务端指定)
bool is_download      # true=下位机->上位机(下载), false=上位机->下位机(上传)
uint8[] file_data     # 文件数据(二进制)

# Response
bool success
string message
int64 file_size

---
")

(setf (get file_transfer::FileTransferResponse :md5sum-) "1f6b3d4d49e3b3db7a6ad1ca13631528")
(setf (get file_transfer::FileTransferResponse :datatype-) "file_transfer/FileTransferResponse")
(setf (get file_transfer::FileTransferResponse :definition-)
      "# Request
string source_path    # 源文件路径(调用方指定)
string target_path    # 目标文件路径(服务端指定)
bool is_download      # true=下位机->上位机(下载), false=上位机->下位机(上传)
uint8[] file_data     # 文件数据(二进制)

# Response
bool success
string message
int64 file_size

---
")



(provide :file_transfer/FileTransfer "1f6b3d4d49e3b3db7a6ad1ca13631528")


