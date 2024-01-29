/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: ssl_wrapper_tracked.proto */

/* Do not generate deprecated warnings for self */
#ifndef PROTOBUF_C__NO_DEPRECATED
#define PROTOBUF_C__NO_DEPRECATED
#endif

#include "ssl_wrapper_tracked.pb-c.h"
void   tracker_wrapper_packet__init
                     (TrackerWrapperPacket         *message)
{
  static const TrackerWrapperPacket init_value = TRACKER_WRAPPER_PACKET__INIT;
  *message = init_value;
}
size_t tracker_wrapper_packet__get_packed_size
                     (const TrackerWrapperPacket *message)
{
  assert(message->base.descriptor == &tracker_wrapper_packet__descriptor);
  return protobuf_c_message_get_packed_size ((const ProtobufCMessage*)(message));
}
size_t tracker_wrapper_packet__pack
                     (const TrackerWrapperPacket *message,
                      uint8_t       *out)
{
  assert(message->base.descriptor == &tracker_wrapper_packet__descriptor);
  return protobuf_c_message_pack ((const ProtobufCMessage*)message, out);
}
size_t tracker_wrapper_packet__pack_to_buffer
                     (const TrackerWrapperPacket *message,
                      ProtobufCBuffer *buffer)
{
  assert(message->base.descriptor == &tracker_wrapper_packet__descriptor);
  return protobuf_c_message_pack_to_buffer ((const ProtobufCMessage*)message, buffer);
}
TrackerWrapperPacket *
       tracker_wrapper_packet__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data)
{
  return (TrackerWrapperPacket *)
     protobuf_c_message_unpack (&tracker_wrapper_packet__descriptor,
                                allocator, len, data);
}
void   tracker_wrapper_packet__free_unpacked
                     (TrackerWrapperPacket *message,
                      ProtobufCAllocator *allocator)
{
  if(!message)
    return;
  assert(message->base.descriptor == &tracker_wrapper_packet__descriptor);
  protobuf_c_message_free_unpacked ((ProtobufCMessage*)message, allocator);
}
static const ProtobufCFieldDescriptor tracker_wrapper_packet__field_descriptors[3] =
{
  {
    "uuid",
    1,
    PROTOBUF_C_LABEL_REQUIRED,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(TrackerWrapperPacket, uuid),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "source_name",
    2,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_STRING,
    0,   /* quantifier_offset */
    offsetof(TrackerWrapperPacket, source_name),
    NULL,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
  {
    "tracked_frame",
    3,
    PROTOBUF_C_LABEL_OPTIONAL,
    PROTOBUF_C_TYPE_MESSAGE,
    0,   /* quantifier_offset */
    offsetof(TrackerWrapperPacket, tracked_frame),
    &tracked_frame__descriptor,
    NULL,
    0,             /* flags */
    0,NULL,NULL    /* reserved1,reserved2, etc */
  },
};
static const unsigned tracker_wrapper_packet__field_indices_by_name[] = {
  1,   /* field[1] = source_name */
  2,   /* field[2] = tracked_frame */
  0,   /* field[0] = uuid */
};
static const ProtobufCIntRange tracker_wrapper_packet__number_ranges[1 + 1] =
{
  { 1, 0 },
  { 0, 3 }
};
const ProtobufCMessageDescriptor tracker_wrapper_packet__descriptor =
{
  PROTOBUF_C__MESSAGE_DESCRIPTOR_MAGIC,
  "TrackerWrapperPacket",
  "TrackerWrapperPacket",
  "TrackerWrapperPacket",
  "",
  sizeof(TrackerWrapperPacket),
  3,
  tracker_wrapper_packet__field_descriptors,
  tracker_wrapper_packet__field_indices_by_name,
  1,  tracker_wrapper_packet__number_ranges,
  (ProtobufCMessageInit) tracker_wrapper_packet__init,
  NULL,NULL,NULL    /* reserved[123] */
};