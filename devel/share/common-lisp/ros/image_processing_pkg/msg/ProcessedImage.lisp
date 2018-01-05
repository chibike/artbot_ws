; Auto-generated. Do not edit!


(cl:in-package image_processing_pkg-msg)


;//! \htmlinclude ProcessedImage.msg.html

(cl:defclass <ProcessedImage> (roslisp-msg-protocol:ros-message)
  ((processed_image
    :reader processed_image
    :initarg :processed_image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (number_of_faces
    :reader number_of_faces
    :initarg :number_of_faces
    :type cl:integer
    :initform 0))
)

(cl:defclass ProcessedImage (<ProcessedImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessedImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessedImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name image_processing_pkg-msg:<ProcessedImage> is deprecated: use image_processing_pkg-msg:ProcessedImage instead.")))

(cl:ensure-generic-function 'processed_image-val :lambda-list '(m))
(cl:defmethod processed_image-val ((m <ProcessedImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing_pkg-msg:processed_image-val is deprecated.  Use image_processing_pkg-msg:processed_image instead.")
  (processed_image m))

(cl:ensure-generic-function 'number_of_faces-val :lambda-list '(m))
(cl:defmethod number_of_faces-val ((m <ProcessedImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader image_processing_pkg-msg:number_of_faces-val is deprecated.  Use image_processing_pkg-msg:number_of_faces instead.")
  (number_of_faces m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessedImage>) ostream)
  "Serializes a message object of type '<ProcessedImage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'processed_image) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number_of_faces)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number_of_faces)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number_of_faces)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number_of_faces)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessedImage>) istream)
  "Deserializes a message object of type '<ProcessedImage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'processed_image) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number_of_faces)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number_of_faces)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number_of_faces)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number_of_faces)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessedImage>)))
  "Returns string type for a message object of type '<ProcessedImage>"
  "image_processing_pkg/ProcessedImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessedImage)))
  "Returns string type for a message object of type 'ProcessedImage"
  "image_processing_pkg/ProcessedImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessedImage>)))
  "Returns md5sum for a message object of type '<ProcessedImage>"
  "4eef9ff0d8d7c418273f14ab4ddf7df6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessedImage)))
  "Returns md5sum for a message object of type 'ProcessedImage"
  "4eef9ff0d8d7c418273f14ab4ddf7df6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessedImage>)))
  "Returns full string definition for message of type '<ProcessedImage>"
  (cl:format cl:nil "sensor_msgs/Image processed_image~%uint32 number_of_faces~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessedImage)))
  "Returns full string definition for message of type 'ProcessedImage"
  (cl:format cl:nil "sensor_msgs/Image processed_image~%uint32 number_of_faces~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessedImage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'processed_image))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessedImage>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessedImage
    (cl:cons ':processed_image (processed_image msg))
    (cl:cons ':number_of_faces (number_of_faces msg))
))
