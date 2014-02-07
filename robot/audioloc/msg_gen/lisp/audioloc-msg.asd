
(cl:in-package :asdf)

(defsystem "audioloc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MultiArrayFloat32" :depends-on ("_package_MultiArrayFloat32"))
    (:file "_package_MultiArrayFloat32" :depends-on ("_package"))
  ))