#include "handle_packet.h"
#include "com.h"
#include <app_netxduo.h>
#include <log.h>
#include <parsed_vision/parsed_vision.pb-c.h>
#include <robot_action/robot_action.pb-c.h>

UINT parse_packet(NX_PACKET *packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch (packet_type) {
  case SSL_WRAPPER: {
    int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
    ParsedFrame *parsed_frame = NULL;
    parsed_frame =
        parsed_frame__unpack(NULL, length, packet->nx_packet_prepend_ptr);
    if (parsed_frame == NULL) {
      LOG_INFO("Protobuf parse failed\r\n");
      ret = NX_INVALID_PACKET;
    } else {
      int ball_x, ball_y, robot_count;
      ball_x = parsed_frame->ball->pos->x;
      ball_y = parsed_frame->ball->pos->x;
      robot_count = parsed_frame->n_robots;
      LOG_INFO("Received msg: Ball (%d, %d), Robots: %d\r\n", ball_x, ball_y, robot_count);

      free(parsed_frame);
    }
    // TODO: we need to extract the interesting data that is supposed
    // to be sent to each robot, then queue them up and send them
  } break;
  case ROBOT_COMMAND: {
    int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
    if (length > 31) {
      ret = NX_INVALID_PACKET;
    } else {
      Command *command = NULL;
      command = command__unpack(NULL, length, packet->nx_packet_prepend_ptr);
      if (command == NULL) {
        ret = NX_INVALID_PACKET;
      } else {
        uint8_t data[32];
        data[0] = MESSAGE_ID_COMMAND;
        memcpy(data + 1, packet->nx_packet_prepend_ptr, length);
        TransmitStatus status;
        if ((status = COM_RF_Transmit(command->robot_id, data, length + 1)) !=
            TRANSMIT_OK) {
          LOG_INFO("Transmit failed: %d\r\n", status);
        }
        protobuf_c_message_free_unpacked(&command->base, NULL);
        static int count = 0;
        LOG_INFO("RF tx len: %d, %d\r\n", length, count);
        ++count;
      }
    }
  } break;
  }

  if (ret != NX_SUCCESS) {
    LOG_WARNING("Failed to parse UDP packet\r\n");
  }

  return ret;
}
