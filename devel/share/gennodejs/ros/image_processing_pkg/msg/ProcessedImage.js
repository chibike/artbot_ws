// Auto-generated. Do not edit!

// (in-package image_processing_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class ProcessedImage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.processed_image = null;
      this.number_of_faces = null;
    }
    else {
      if (initObj.hasOwnProperty('processed_image')) {
        this.processed_image = initObj.processed_image
      }
      else {
        this.processed_image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('number_of_faces')) {
        this.number_of_faces = initObj.number_of_faces
      }
      else {
        this.number_of_faces = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProcessedImage
    // Serialize message field [processed_image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.processed_image, buffer, bufferOffset);
    // Serialize message field [number_of_faces]
    bufferOffset = _serializer.uint32(obj.number_of_faces, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProcessedImage
    let len;
    let data = new ProcessedImage(null);
    // Deserialize message field [processed_image]
    data.processed_image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [number_of_faces]
    data.number_of_faces = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.processed_image);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'image_processing_pkg/ProcessedImage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4eef9ff0d8d7c418273f14ab4ddf7df6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/Image processed_image
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProcessedImage(null);
    if (msg.processed_image !== undefined) {
      resolved.processed_image = sensor_msgs.msg.Image.Resolve(msg.processed_image)
    }
    else {
      resolved.processed_image = new sensor_msgs.msg.Image()
    }

    if (msg.number_of_faces !== undefined) {
      resolved.number_of_faces = msg.number_of_faces;
    }
    else {
      resolved.number_of_faces = 0
    }

    return resolved;
    }
};

module.exports = ProcessedImage;
