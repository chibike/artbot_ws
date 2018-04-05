
(cl:in-package :asdf)

(defsystem "image_processing_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ProcessedImage" :depends-on ("_package_ProcessedImage"))
    (:file "_package_ProcessedImage" :depends-on ("_package"))
    (:file "StateChangeRequestAction" :depends-on ("_package_StateChangeRequestAction"))
    (:file "_package_StateChangeRequestAction" :depends-on ("_package"))
    (:file "StateChangeRequestActionFeedback" :depends-on ("_package_StateChangeRequestActionFeedback"))
    (:file "_package_StateChangeRequestActionFeedback" :depends-on ("_package"))
    (:file "StateChangeRequestActionGoal" :depends-on ("_package_StateChangeRequestActionGoal"))
    (:file "_package_StateChangeRequestActionGoal" :depends-on ("_package"))
    (:file "StateChangeRequestActionResult" :depends-on ("_package_StateChangeRequestActionResult"))
    (:file "_package_StateChangeRequestActionResult" :depends-on ("_package"))
    (:file "StateChangeRequestFeedback" :depends-on ("_package_StateChangeRequestFeedback"))
    (:file "_package_StateChangeRequestFeedback" :depends-on ("_package"))
    (:file "StateChangeRequestGoal" :depends-on ("_package_StateChangeRequestGoal"))
    (:file "_package_StateChangeRequestGoal" :depends-on ("_package"))
    (:file "StateChangeRequestResult" :depends-on ("_package_StateChangeRequestResult"))
    (:file "_package_StateChangeRequestResult" :depends-on ("_package"))
  ))