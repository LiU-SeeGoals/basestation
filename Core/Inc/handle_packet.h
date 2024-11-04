#ifndef __APP_HANDLE_PACKET_H__
#define __APP_HANDLE_PACKET_H__
#include <nx_api.h>
#include <stdint.h>

enum _PACKET_TYPE {SSL_WRAPPER, ROBOT_COMMAND, PACKET_TYPE_COUNT};
typedef enum _PACKET_TYPE PACKET_TYPE;

extern UINT parse_packet(uint8_t* packet, uint16_t length,
                         PACKET_TYPE packet_type);

#endif // __APP_HANDLE_PACKET_H__s
