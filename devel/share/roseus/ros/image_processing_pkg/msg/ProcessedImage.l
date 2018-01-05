;; Auto-generated. Do not edit!


(when (boundp 'image_processing_pkg::ProcessedImage)
  (if (not (find-package "IMAGE_PROCESSING_PKG"))
    (make-package "IMAGE_PROCESSING_PKG"))
  (shadow 'ProcessedImage (find-package "IMAGE_PROCESSING_PKG")))
(unless (find-package "IMAGE_PROCESSING_PKG::PROCESSEDIMAGE")
  (make-package "IMAGE_PROCESSING_PKG::PROCESSEDIMAGE"))

(in-package "ROS")
;;//! \htmlinclude ProcessedImage.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass image_processing_pkg::ProcessedImage
  :super ros::object
  :slots (_processed_image _number_of_faces ))

(defmethod image_processing_pkg::ProcessedImage
  (:init
   (&key
    ((:processed_image __processed_image) (instance sensor_msgs::Image :init))
    ((:number_of_faces __number_of_faces) 0)
    )
   (send-super :init)
   (setq _processed_image __processed_image)
   (setq _number_of_faces (round __number_of_faces))
   self)
  (:processed_image
   (&rest __processed_image)
   (if (keywordp (car __processed_image))
       (send* _processed_image __processed_image)
     (progn
       (if __processed_image (setq _processed_image (car __processed_image)))
       _processed_image)))
  (:number_of_faces
   (&optional __number_of_faces)
   (if __number_of_faces (setq _number_of_faces __number_of_faces)) _number_of_faces)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _processed_image
    (send _processed_image :serialization-length)
    ;; uint32 _number_of_faces
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _processed_image
       (send _processed_image :serialize s)
     ;; uint32 _number_of_faces
       (write-long _number_of_faces s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _processed_image
     (send _processed_image :deserialize buf ptr-) (incf ptr- (send _processed_image :serialization-length))
   ;; uint32 _number_of_faces
     (setq _number_of_faces (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get image_processing_pkg::ProcessedImage :md5sum-) "4eef9ff0d8d7c418273f14ab4ddf7df6")
(setf (get image_processing_pkg::ProcessedImage :datatype-) "image_processing_pkg/ProcessedImage")
(setf (get image_processing_pkg::ProcessedImage :definition-)
      "sensor_msgs/Image processed_image
uint32 number_of_faces
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :image_processing_pkg/ProcessedImage "4eef9ff0d8d7c418273f14ab4ddf7df6")


