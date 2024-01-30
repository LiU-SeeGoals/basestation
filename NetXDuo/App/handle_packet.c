#include "handle_packet.h"
#include <nrf24l01.h>

UINT parse_packet(NX_PACKET* packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch(packet_type) {
    case SSL_WRAPPER:
      {
        int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
        SSLWrapperPacket* ssl_packet = NULL;
        ssl_packet = ssl__wrapper_packet__unpack(NULL, length, packet->nx_packet_prepend_ptr);
        if (ssl_packet == NULL) {
          ret = NX_INVALID_PACKET;
        } else {
          printf("[NX] received msg\r\n");
        }
        // TODO: we need to extract the interesting data that is supposed
        // to be sent to each robot, then queue them up and send them
      }
      break;
    case ROBOT_COMMAND:
      {
        int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
        if (length > 32) {
          ret = NX_INVALID_PACKET;
        } else {
          NRF_EnterMode(NRF_MODE_STANDBY1);
          NRF_Transmit(packet->nx_packet_prepend_ptr, length);
          printf("[NX] RF tx len: %d\r\n", length);
          NRF_EnterMode(NRF_MODE_RX);
        }
      }
      break;
  }

  if (ret != NX_SUCCESS) {
    printf("[NX] Failed to parse UDP packet\r\n");
  }

  return ret;
}
