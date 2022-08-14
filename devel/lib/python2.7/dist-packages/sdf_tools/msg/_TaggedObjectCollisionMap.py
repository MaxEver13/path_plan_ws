# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from sdf_tools/TaggedObjectCollisionMap.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class TaggedObjectCollisionMap(genpy.Message):
  _md5sum = "320371317f699b0048968a467deb0a13"
  _type = "sdf_tools/TaggedObjectCollisionMap"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
geometry_msgs/Transform origin_transform
geometry_msgs/Vector3 dimensions
float64 cell_size
uint32 number_of_components
bool components_valid
bool convex_segments_valid
bool initialized
uint8[] OOB_value
uint8[] data

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

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['header','origin_transform','dimensions','cell_size','number_of_components','components_valid','convex_segments_valid','initialized','OOB_value','data']
  _slot_types = ['std_msgs/Header','geometry_msgs/Transform','geometry_msgs/Vector3','float64','uint32','bool','bool','bool','uint8[]','uint8[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,origin_transform,dimensions,cell_size,number_of_components,components_valid,convex_segments_valid,initialized,OOB_value,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TaggedObjectCollisionMap, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.origin_transform is None:
        self.origin_transform = geometry_msgs.msg.Transform()
      if self.dimensions is None:
        self.dimensions = geometry_msgs.msg.Vector3()
      if self.cell_size is None:
        self.cell_size = 0.
      if self.number_of_components is None:
        self.number_of_components = 0
      if self.components_valid is None:
        self.components_valid = False
      if self.convex_segments_valid is None:
        self.convex_segments_valid = False
      if self.initialized is None:
        self.initialized = False
      if self.OOB_value is None:
        self.OOB_value = b''
      if self.data is None:
        self.data = b''
    else:
      self.header = std_msgs.msg.Header()
      self.origin_transform = geometry_msgs.msg.Transform()
      self.dimensions = geometry_msgs.msg.Vector3()
      self.cell_size = 0.
      self.number_of_components = 0
      self.components_valid = False
      self.convex_segments_valid = False
      self.initialized = False
      self.OOB_value = b''
      self.data = b''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_11dI3B().pack(_x.origin_transform.translation.x, _x.origin_transform.translation.y, _x.origin_transform.translation.z, _x.origin_transform.rotation.x, _x.origin_transform.rotation.y, _x.origin_transform.rotation.z, _x.origin_transform.rotation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.cell_size, _x.number_of_components, _x.components_valid, _x.convex_segments_valid, _x.initialized))
      _x = self.OOB_value
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.origin_transform is None:
        self.origin_transform = geometry_msgs.msg.Transform()
      if self.dimensions is None:
        self.dimensions = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 95
      (_x.origin_transform.translation.x, _x.origin_transform.translation.y, _x.origin_transform.translation.z, _x.origin_transform.rotation.x, _x.origin_transform.rotation.y, _x.origin_transform.rotation.z, _x.origin_transform.rotation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.cell_size, _x.number_of_components, _x.components_valid, _x.convex_segments_valid, _x.initialized,) = _get_struct_11dI3B().unpack(str[start:end])
      self.components_valid = bool(self.components_valid)
      self.convex_segments_valid = bool(self.convex_segments_valid)
      self.initialized = bool(self.initialized)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.OOB_value = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_11dI3B().pack(_x.origin_transform.translation.x, _x.origin_transform.translation.y, _x.origin_transform.translation.z, _x.origin_transform.rotation.x, _x.origin_transform.rotation.y, _x.origin_transform.rotation.z, _x.origin_transform.rotation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.cell_size, _x.number_of_components, _x.components_valid, _x.convex_segments_valid, _x.initialized))
      _x = self.OOB_value
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.origin_transform is None:
        self.origin_transform = geometry_msgs.msg.Transform()
      if self.dimensions is None:
        self.dimensions = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 95
      (_x.origin_transform.translation.x, _x.origin_transform.translation.y, _x.origin_transform.translation.z, _x.origin_transform.rotation.x, _x.origin_transform.rotation.y, _x.origin_transform.rotation.z, _x.origin_transform.rotation.w, _x.dimensions.x, _x.dimensions.y, _x.dimensions.z, _x.cell_size, _x.number_of_components, _x.components_valid, _x.convex_segments_valid, _x.initialized,) = _get_struct_11dI3B().unpack(str[start:end])
      self.components_valid = bool(self.components_valid)
      self.convex_segments_valid = bool(self.convex_segments_valid)
      self.initialized = bool(self.initialized)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.OOB_value = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_11dI3B = None
def _get_struct_11dI3B():
    global _struct_11dI3B
    if _struct_11dI3B is None:
        _struct_11dI3B = struct.Struct("<11dI3B")
    return _struct_11dI3B
