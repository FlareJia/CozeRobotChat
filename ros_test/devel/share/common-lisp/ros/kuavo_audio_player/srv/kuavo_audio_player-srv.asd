
(cl:in-package :asdf)

(defsystem "kuavo_audio_player-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "playmusic" :depends-on ("_package_playmusic"))
    (:file "_package_playmusic" :depends-on ("_package"))
  ))