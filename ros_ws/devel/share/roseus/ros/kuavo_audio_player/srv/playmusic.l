;; Auto-generated. Do not edit!


(when (boundp 'kuavo_audio_player::playmusic)
  (if (not (find-package "KUAVO_AUDIO_PLAYER"))
    (make-package "KUAVO_AUDIO_PLAYER"))
  (shadow 'playmusic (find-package "KUAVO_AUDIO_PLAYER")))
(unless (find-package "KUAVO_AUDIO_PLAYER::PLAYMUSIC")
  (make-package "KUAVO_AUDIO_PLAYER::PLAYMUSIC"))
(unless (find-package "KUAVO_AUDIO_PLAYER::PLAYMUSICREQUEST")
  (make-package "KUAVO_AUDIO_PLAYER::PLAYMUSICREQUEST"))
(unless (find-package "KUAVO_AUDIO_PLAYER::PLAYMUSICRESPONSE")
  (make-package "KUAVO_AUDIO_PLAYER::PLAYMUSICRESPONSE"))

(in-package "ROS")





(defclass kuavo_audio_player::playmusicRequest
  :super ros::object
  :slots (_music_path ))

(defmethod kuavo_audio_player::playmusicRequest
  (:init
   (&key
    ((:music_path __music_path) "")
    )
   (send-super :init)
   (setq _music_path (string __music_path))
   self)
  (:music_path
   (&optional __music_path)
   (if __music_path (setq _music_path __music_path)) _music_path)
  (:serialization-length
   ()
   (+
    ;; string _music_path
    4 (length _music_path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _music_path
       (write-long (length _music_path) s) (princ _music_path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _music_path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _music_path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kuavo_audio_player::playmusicResponse
  :super ros::object
  :slots (_success_flag ))

(defmethod kuavo_audio_player::playmusicResponse
  (:init
   (&key
    ((:success_flag __success_flag) nil)
    )
   (send-super :init)
   (setq _success_flag __success_flag)
   self)
  (:success_flag
   (&optional (__success_flag :null))
   (if (not (eq __success_flag :null)) (setq _success_flag __success_flag)) _success_flag)
  (:serialization-length
   ()
   (+
    ;; bool _success_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success_flag
       (if _success_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success_flag
     (setq _success_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass kuavo_audio_player::playmusic
  :super ros::object
  :slots ())

(setf (get kuavo_audio_player::playmusic :md5sum-) "2b45fccbddec6f75379e3c0de524323f")
(setf (get kuavo_audio_player::playmusic :datatype-) "kuavo_audio_player/playmusic")
(setf (get kuavo_audio_player::playmusic :request) kuavo_audio_player::playmusicRequest)
(setf (get kuavo_audio_player::playmusic :response) kuavo_audio_player::playmusicResponse)

(defmethod kuavo_audio_player::playmusicRequest
  (:response () (instance kuavo_audio_player::playmusicResponse :init)))

(setf (get kuavo_audio_player::playmusicRequest :md5sum-) "2b45fccbddec6f75379e3c0de524323f")
(setf (get kuavo_audio_player::playmusicRequest :datatype-) "kuavo_audio_player/playmusicRequest")
(setf (get kuavo_audio_player::playmusicRequest :definition-)
      "string music_path
---
bool success_flag

")

(setf (get kuavo_audio_player::playmusicResponse :md5sum-) "2b45fccbddec6f75379e3c0de524323f")
(setf (get kuavo_audio_player::playmusicResponse :datatype-) "kuavo_audio_player/playmusicResponse")
(setf (get kuavo_audio_player::playmusicResponse :definition-)
      "string music_path
---
bool success_flag

")



(provide :kuavo_audio_player/playmusic "2b45fccbddec6f75379e3c0de524323f")


