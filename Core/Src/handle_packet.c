#include "handle_packet.h"
#include "com.h"
#include <nrf24l01.h>
#include <protobuf-c/protobuf-c.h>
#include <protobuf-c/robot_action.pb-c.h>
#include <protobuf-c/parsed_vision.pb-c.h>

UINT parse_packet(NX_PACKET* packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch(packet_type) {
    case SSL_WRAPPER:
      {
        int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
        ParsedFrame* prased_frame = NULL;
        prased_frame = parsed_frame__unpack(NULL, length, packet->nx_packet_prepend_ptr);
        if (prased_frame == NULL) {
          ret = NX_INVALID_PACKET;
        } else {
          printf("[NX] received msg\r\n");

          free(prased_frame);
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
          Action__Command* command = NULL;
          command = action__command__unpack(NULL, length, packet->nx_packet_prepend_ptr);
          if (command == NULL) {
            ret = NX_INVALID_PACKET;
          } else {
            uint8_t address[5] = ROBOT_ACTION_ADDR(command->robot_id);
            NRF_EnterMode(NRF_MODE_STANDBY1);
            NRF_WriteRegister(NRF_REG_TX_ADDR, address, 5);
            NRF_Transmit(packet->nx_packet_prepend_ptr, length);
            /*for (int i = 0; i < length; ++i) {
                printf("%u,", packet->nx_packet_prepend_ptr[i]);
            }
            printf("\n");*/
            printf("[NX] RF tx len: %d\r\n", length);
            NRF_EnterMode(NRF_MODE_RX);
          }
        }
      }
    break;
  }

  if (ret != NX_SUCCESS) {
    printf("[NX] Failed to parse UDP packet\r\n");
  }

  return ret;
}
