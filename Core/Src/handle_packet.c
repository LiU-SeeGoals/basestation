#include "handle_packet.h"
#include "com.h"
#include <app_netxduo.h>
#include <log.h>
#include <parsed_vision/parsed_vision.pb-c.h>
#include <robot_action/robot_action.pb-c.h>

UINT parse_packet(uint8_t* packet, uint16_t len, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch (packet_type) {
  case SSL_WRAPPER: {
    ParsedFrame *parsed_frame = NULL;
    parsed_frame =
        parsed_frame__unpack(NULL, len, packet);
    if (parsed_frame == NULL) {
      LOG_INFO("Protobuf parse failed\r\n");
      ret = NX_INVALID_PACKET;
    } else {
      static int msg_recieved = 0;
      int32_t ball_x, ball_y, robot_count;
      ball_x = parsed_frame->ball->pos->x;
      ball_y = parsed_frame->ball->pos->y;
      robot_count = parsed_frame->n_robots;

      ++msg_recieved;
      LOG_INFO("Received msg(%d): Ball (%d, %d), Robots: %d\r\n", msg_recieved, ball_x, ball_y, robot_count);

      uint8_t msg[1 + 2 * sizeof(int32_t)];
      msg[0] = MESSAGE_ID_VISION;
      memcpy(msg + 1, &ball_x, sizeof(int32_t));
      memcpy(msg + 1 + sizeof(int32_t), &ball_y, sizeof(int32_t));
      COM_RF_Broadcast(msg, sizeof(msg));

      for (int32_t i = 0; i < robot_count; ++i) {
        uint8_t msg[1 + 4 * sizeof(int32_t) + 2 * sizeof(float)];
        msg[0] = MESSAGE_ID_VISION;

        struct {
          int32_t x, y;
          float w;
          int32_t dx, dy;
          float dw;
        } pos;

        pos.x = parsed_frame->robots[i]->pos->x;
        pos.y = parsed_frame->robots[i]->pos->y;
        pos.w = parsed_frame->robots[i]->pos->w;
        pos.dx = parsed_frame->robots[i]->vel->x;
        pos.dy = parsed_frame->robots[i]->vel->y;
        pos.dw = parsed_frame->robots[i]->vel->w;
        memcpy(msg + 1, &pos, sizeof(pos));
        uint8_t id = parsed_frame->robots[i]->robot_id;
        if (connected_robots[id] == ROBOT_CONNECTED) {
          uint8_t id = parsed_frame->robots[i]->robot_id;
          uint32_t start = HAL_GetTick();
          TransmitStatus status = COM_RF_Transmit(id, msg, sizeof(msg));
          if (status != TRANSMIT_OK) {
            LOG_INFO("Transmit failed: %d\n", status);
          }
          uint32_t end = HAL_GetTick();
          LOG_INFO("Time passed: %u\n", end - start);
        }
      }

      free(parsed_frame);
    }
  } break;
  case ROBOT_COMMAND: {
    if (len > 31) {
      ret = NX_INVALID_PACKET;
    } else {
      Command *command = NULL;
      command = command__unpack(NULL, len, packet);
      if (command == NULL) {
        ret = NX_INVALID_PACKET;
      } else {
        uint8_t data[32];
        data[0] = MESSAGE_ID_COMMAND;
        memcpy(data + 1, packet, len);
        TransmitStatus status;
        if ((status = COM_RF_Transmit(command->robot_id, data, len + 1)) !=
            TRANSMIT_OK) {
          LOG_INFO("Transmit failed: %d\r\n", status);
        }
        protobuf_c_message_free_unpacked(&command->base, NULL);
        static int count = 0;
        LOG_INFO("RF tx len: %d, %d\r\n", len, count);
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
