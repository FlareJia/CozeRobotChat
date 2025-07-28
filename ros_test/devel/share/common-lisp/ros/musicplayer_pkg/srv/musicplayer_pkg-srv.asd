
(cl:in-package :asdf)

(defsystem "musicplayer_pkg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "playmusic" :depends-on ("_package_playmusic"))
    (:file "_package_playmusic" :depends-on ("_package"))
  ))