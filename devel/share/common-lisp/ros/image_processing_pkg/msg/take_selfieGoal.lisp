; Auto-generated. Do not edit!


(cl:in-package image_processing_pkg-msg)


;//! \htmlinclude take_selfieGoal.msg.html

(cl:defclass <take_selfieGoal> (roslisp-msg-protocol:ros-message)
  ((order
    :reader order
    :initarg :order
    :type cl:integer
    :initform 0))
)

(cl:defclass take_selfieGoal (<take_selfieGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <take_selfieGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'take_selfieGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_processing_pkg-msg:<take_selfieGoal> is deprecated: use image_processing_pkg-msg:take_selfieGoal instead.")))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <take_selfieGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing_pkg-msg:order-val is deprecated.  Use image_processing_pkg-msg:order instead.")
  (order m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <take_selfieGoal>) ostream)
  "Serializes a message object of type '<take_selfieGoal>"
  (cl:let* ((signed (cl:slot-value msg 'order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <take_selfieGoal>) istream)
  "Deserializes a message object of type '<take_selfieGoal>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<take_selfieGoal>)))
  "Returns string type for a message object of type '<take_selfieGoal>"
  "image_processing_pkg/take_selfieGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'take_selfieGoal)))
  "Returns string type for a message object of type 'take_selfieGoal"
  "image_processing_pkg/take_selfieGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<take_selfieGoal>)))
  "Returns md5sum for a message object of type '<take_selfieGoal>"
  "6889063349a00b249bd1661df429d822")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'take_selfieGoal)))
  "Returns md5sum for a message object of type 'take_selfieGoal"
  "6889063349a00b249bd1661df429d822")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<take_selfieGoal>)))
  "Returns full string definition for message of type '<take_selfieGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int32 order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'take_selfieGoal)))
  "Returns full string definition for message of type 'take_selfieGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int32 order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <take_selfieGoal>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <take_selfieGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'take_selfieGoal
    (cl:cons ':order (order msg))
))