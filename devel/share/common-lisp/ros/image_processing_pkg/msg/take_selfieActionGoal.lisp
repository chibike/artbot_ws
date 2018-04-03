; Auto-generated. Do not edit!


(cl:in-package image_processing_pkg-msg)


;//! \htmlinclude take_selfieActionGoal.msg.html

(cl:defclass <take_selfieActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type image_processing_pkg-msg:take_selfieGoal
    :initform (cl:make-instance 'image_processing_pkg-msg:take_selfieGoal)))
)

(cl:defclass take_selfieActionGoal (<take_selfieActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <take_selfieActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'take_selfieActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_processing_pkg-msg:<take_selfieActionGoal> is deprecated: use image_processing_pkg-msg:take_selfieActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <take_selfieActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing_pkg-msg:header-val is deprecated.  Use image_processing_pkg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <take_selfieActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing_pkg-msg:goal_id-val is deprecated.  Use image_processing_pkg-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <take_selfieActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing_pkg-msg:goal-val is deprecated.  Use image_processing_pkg-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <take_selfieActionGoal>) ostream)
  "Serializes a message object of type '<take_selfieActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <take_selfieActionGoal>) istream)
  "Deserializes a message object of type '<take_selfieActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<take_selfieActionGoal>)))
  "Returns string type for a message object of type '<take_selfieActionGoal>"
  "image_processing_pkg/take_selfieActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'take_selfieActionGoal)))
  "Returns string type for a message object of type 'take_selfieActionGoal"
  "image_processing_pkg/take_selfieActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<take_selfieActionGoal>)))
  "Returns md5sum for a message object of type '<take_selfieActionGoal>"
  "006871c7fa1d0e3d5fe2226bf17b2a94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'take_selfieActionGoal)))
  "Returns md5sum for a message object of type 'take_selfieActionGoal"
  "006871c7fa1d0e3d5fe2226bf17b2a94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<take_selfieActionGoal>)))
  "Returns full string definition for message of type '<take_selfieActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%take_selfieGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: image_processing_pkg/take_selfieGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int32 order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'take_selfieActionGoal)))
  "Returns full string definition for message of type 'take_selfieActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%take_selfieGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: image_processing_pkg/take_selfieGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int32 order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <take_selfieActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <take_selfieActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'take_selfieActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
