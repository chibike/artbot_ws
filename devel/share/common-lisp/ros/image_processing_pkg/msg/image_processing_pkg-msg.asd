
(cl:in-package :asdf)

(defsystem "image_processing_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ProcessedImage" :depends-on ("_package_ProcessedImage"))
    (:file "_package_ProcessedImage" :depends-on ("_package"))
  ))