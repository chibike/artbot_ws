
(cl:in-package :asdf)

(defsystem "image_processing_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ProcessedImage" :depends-on ("_package_ProcessedImage"))
    (:file "_package_ProcessedImage" :depends-on ("_package"))
    (:file "take_selfieAction" :depends-on ("_package_take_selfieAction"))
    (:file "_package_take_selfieAction" :depends-on ("_package"))
    (:file "take_selfieActionFeedback" :depends-on ("_package_take_selfieActionFeedback"))
    (:file "_package_take_selfieActionFeedback" :depends-on ("_package"))
    (:file "take_selfieActionGoal" :depends-on ("_package_take_selfieActionGoal"))
    (:file "_package_take_selfieActionGoal" :depends-on ("_package"))
    (:file "take_selfieActionResult" :depends-on ("_package_take_selfieActionResult"))
    (:file "_package_take_selfieActionResult" :depends-on ("_package"))
    (:file "take_selfieFeedback" :depends-on ("_package_take_selfieFeedback"))
    (:file "_package_take_selfieFeedback" :depends-on ("_package"))
    (:file "take_selfieGoal" :depends-on ("_package_take_selfieGoal"))
    (:file "_package_take_selfieGoal" :depends-on ("_package"))
    (:file "take_selfieResult" :depends-on ("_package_take_selfieResult"))
    (:file "_package_take_selfieResult" :depends-on ("_package"))
  ))