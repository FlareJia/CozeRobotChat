
(cl:in-package :asdf)

(defsystem "file_transfer-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FileTransfer" :depends-on ("_package_FileTransfer"))
    (:file "_package_FileTransfer" :depends-on ("_package"))
  ))