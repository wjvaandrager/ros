"""autogenerated by genpy from rosserial_msgs/TopicInfo.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TopicInfo(genpy.Message):
  _md5sum = "af972f89eab8180c74022cbaed55d206"
  _type = "rosserial_msgs/TopicInfo"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#The topic_id defines the communication endpoint for this type of message
uint16 topic_id

uint16 ID_PUBLISHER=0
uint16 ID_SUBSCRIBER=1
uint16 ID_SERVICE_SERVER=2
uint16 ID_SERVICE_CLIENT=3
uint16 ID_PARAMETER_REQUEST=4
uint16 ID_LOG=5
uint16 ID_TIME =10


#any topic_id > 100 is a dynamically registered/advertised endpoint
#ie. it is the result of a subscribe, publish, or advertise

string topic_name

string message_type

"""
  # Pseudo-constants
  ID_PUBLISHER = 0
  ID_SUBSCRIBER = 1
  ID_SERVICE_SERVER = 2
  ID_SERVICE_CLIENT = 3
  ID_PARAMETER_REQUEST = 4
  ID_LOG = 5
  ID_TIME = 10

  __slots__ = ['topic_id','topic_name','message_type']
  _slot_types = ['uint16','string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       topic_id,topic_name,message_type

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TopicInfo, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.topic_id is None:
        self.topic_id = 0
      if self.topic_name is None:
        self.topic_name = ''
      if self.message_type is None:
        self.message_type = ''
    else:
      self.topic_id = 0
      self.topic_name = ''
      self.message_type = ''

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
      buff.write(_struct_H.pack(self.topic_id))
      _x = self.topic_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.message_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 2
      (self.topic_id,) = _struct_H.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.topic_name = str[start:end].decode('utf-8')
      else:
        self.topic_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.message_type = str[start:end].decode('utf-8')
      else:
        self.message_type = str[start:end]
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
      buff.write(_struct_H.pack(self.topic_id))
      _x = self.topic_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.message_type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 2
      (self.topic_id,) = _struct_H.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.topic_name = str[start:end].decode('utf-8')
      else:
        self.topic_name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.message_type = str[start:end].decode('utf-8')
      else:
        self.message_type = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_H = struct.Struct("<H")