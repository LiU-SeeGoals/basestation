/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9-dev */

#ifndef PB_SSL_WRAPPER_TRACKED_PB_H_INCLUDED
#define PB_SSL_WRAPPER_TRACKED_PB_H_INCLUDED
#include <pb.h>
#include "ssl_detection_tracked.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
/* A wrapper packet containing meta data of the source
 Also serves for the possibility to extend the protocol later */
typedef struct _TrackerWrapperPacket {
    /* A random UUID of the source that is kept constant at the source while running
 If multiple sources are broadcasting to the same network, this id can be used to identify individual sources */
    pb_callback_t uuid;
    /* The name of the source software that is producing this messages. */
    pb_callback_t source_name;
    /* The tracked frame */
    bool has_tracked_frame;
    TrackedFrame tracked_frame;
} TrackerWrapperPacket;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define TrackerWrapperPacket_init_default        {{{NULL}, NULL}, {{NULL}, NULL}, false, TrackedFrame_init_default}
#define TrackerWrapperPacket_init_zero           {{{NULL}, NULL}, {{NULL}, NULL}, false, TrackedFrame_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define TrackerWrapperPacket_uuid_tag            1
#define TrackerWrapperPacket_source_name_tag     2
#define TrackerWrapperPacket_tracked_frame_tag   3

/* Struct field encoding specification for nanopb */
#define TrackerWrapperPacket_FIELDLIST(X, a) \
X(a, CALLBACK, REQUIRED, STRING,   uuid,              1) \
X(a, CALLBACK, OPTIONAL, STRING,   source_name,       2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  tracked_frame,     3)
#define TrackerWrapperPacket_CALLBACK pb_default_field_callback
#define TrackerWrapperPacket_DEFAULT NULL
#define TrackerWrapperPacket_tracked_frame_MSGTYPE TrackedFrame

extern const pb_msgdesc_t TrackerWrapperPacket_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define TrackerWrapperPacket_fields &TrackerWrapperPacket_msg

/* Maximum encoded size of messages (where known) */
/* TrackerWrapperPacket_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif