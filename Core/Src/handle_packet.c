#include "handle_packet.h"
#include "com.h"
#include <app_netxduo.h>
#include <log.h>
#include <parsed_vision/parsed_vision.pb-c.h>
#include <robot_action/robot_action.pb-c.h>

#define MAX_NO_RESPONSES 5
uint8_t no_robot_responses[MAX_ROBOT_COUNT];

UINT parse_packet(NX_PACKET *packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch (packet_type) {
  case SSL_WRAPPER: {
    int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
    ParsedFrame *prased_frame = NULL;
    prased_frame = parsed_frame__unpack(NULL, length, packet->nx_packet_prepend_ptr);
    if (prased_frame == NULL) {
      ret = NX_INVALID_PACKET;
    } else {
      LOG_INFO("Received msg\r\n");

      free(prased_frame);
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
        LOG_ERROR("Invalid packet!\r\n");
      } else {
        const ProtobufCEnumValue* enum_value = protobuf_c_enum_descriptor_get_value(&action_type__descriptor, command->command_id);

        if (connected_robots[command->robot_id] == ROBOT_CONNECTED) {
          uint8_t data[32];
          data[0] = 1;
          memcpy(data + 1, packet->nx_packet_prepend_ptr, length);
          TransmitStatus status = COM_RF_Transmit(command->robot_id, data, length + 1);
          if (status != TRANSMIT_OK) {
            LOG_INFO("Failed sending robot #%d command %s\r\n", command->robot_id, enum_value->name);
            no_robot_responses[command->robot_id]++;

            if (no_robot_responses[command->robot_id] > MAX_NO_RESPONSES) {
              connected_robots[command->robot_id] = ROBOT_DISCONNECTED;
              no_robot_responses[command->robot_id] = 0;
              LOG_INFO("Disconnected robot #%d after %d TX attempts\r\n", command->robot_id, MAX_NO_RESPONSES+1);
            }
          } else {
            no_robot_responses[command->robot_id] = 0;
            LOG_INFO("Successfully sent robot #%d command %s\r\n", command->robot_id, enum_value->name);
          }
        } else {
          LOG_INFO("Robot #%d is disconnected, can't send %s\r\n", command->robot_id, enum_value->name);
        }
      }

      protobuf_c_message_free_unpacked(&command->base, NULL);
    }
  } break;
  }

  if (ret != NX_SUCCESS) {
    LOG_WARNING("Failed to parse UDP packet\r\n");
  }

  return ret;
}
