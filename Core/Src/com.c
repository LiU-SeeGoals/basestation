/* Private includes */
#include "com.h"
#include <nrf24l01.h>
#include <nrf_helper_defines.h>
#include <log.h>
#include <protobuf-c/protobuf-c.h>
#include <robot_action/robot_action.pb-c.h>
#include <parsed_vision/parsed_vision.pb-c.h>

/* Private defines */
#define PIPE_CONTROLLER 0
#define PIPE_VISION     1
#define CONNECT_MAGIC   0x4d, 0xf8, 0x42, 0x79
#define MAX_NO_RESPONSES 1000

/* Private variables */
uint8_t no_robot_responses[MAX_ROBOT_COUNT];
volatile RobotConnection connected_robots[MAX_ROBOT_COUNT];
TX_SEMAPHORE semaphore;
volatile TransmitStatus com_ack; // Status of acknowledgement of a transmission
static LOG_Module internal_log_mod;
uint8_t msg_order[MAX_ROBOT_COUNT];


/*
 * Public functions implementations
 */

void COM_RF_Init(SPI_HandleTypeDef* hspi) {
  LOG_InitModule(&internal_log_mod, "RF", LOG_LEVEL_TRACE, 0);
  uint8_t address[5] = CONTROLLER_ADDR;
  com_ack = TRANSMIT_OK;
  if (tx_semaphore_create(&semaphore, "NRF-semaphore", 1) != TX_SUCCESS) {
    LOG_ERROR("Failed creating NRF-semaphore\r\n");
  }

  // Mark all robots as disconnected
  for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
    connected_robots[i] = ROBOT_DISCONNECTED;
    msg_order[i] = 0;
  }

  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    LOG_WARNING("Couldn't verify nRF24 SPI...\r\n");
  }

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // See nRF24L01+ chapter 6.3 for more...
  // Set the RF channel frequency to 2500, i.e. outside of wifi range
  // It's defined as: 2400 + NRF_REG_RF_CH [MHz]
  // NRF_REG_RF_CH can 0-127, but not all values seem to work
  //NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x64); // 2500 works
  //NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x70); // 2512 works
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x7d); // 2525
  //NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x7f); // 2527 works

  // Setup the TX address.
  // We also have to set pipe 0 to receive on the same address.
  uint8_t send_addr[5] = ROBOT_ACTION_ADDR(0);

  NRF_WriteRegister(NRF_REG_TX_ADDR, send_addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, send_addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, address, 5);

  // We enable ACK payloads which needs dynamic payload to function.
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_ACK_PAY);
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_DPL);
  NRF_WriteRegisterByte(NRF_REG_DYNPD, 0x03);

  // Setup for 3 max retries when sending and 500 us between each retry.
  // For motivation, see page 60 in datasheet.
  NRF_WriteRegisterByte(NRF_REG_SETUP_RETR, 0x13);

  // Enter receive mode
  NRF_EnterMode(NRF_MODE_RX);
  LOG_INFO("Initialized...\r\n");
}

static uint8_t msg[] = {0, 'P', 'i', 'n', 'g'};

void COM_RF_PingRobots(bool ping_all) {
  LOG_INFO("Requesting ping...\r\n");
  for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
    if (!ping_all && connected_robots[i] != ROBOT_CONNECTED) {
      continue;
    }

    // Request a ping message up to 2 times.
    // This is because the robot might ignore the first one if the message id
    // happens to match the last value the robot knows.
    for (int k = 0; k < 2 ; ++k) {
      connected_robots[i] = ROBOT_PENDING;
      if (COM_RF_Transmit(i, msg, 5) != TRANSMIT_OK) {
        LOG_INFO("Robot %d did not respond\r\n", i);
        connected_robots[i] = ROBOT_DISCONNECTED;
        goto next;
      }
      for (int j = 0; j < 50; ++j) {
        tx_thread_sleep(1);
        if (connected_robots[i] == ROBOT_CONNECTED) {
          LOG_INFO("Robot %d responded\r\n", i);
          goto next;
        }
      }
    }

    // Coming here means both ack:s were received, but the robot still did not respond.
    LOG_WARNING("Robot %d receives messages, but is not responding to pings.\r\n", i);

    connected_robots[i] = ROBOT_DISCONNECTED;
    next:;
  }
}

void COM_RF_HandleIRQ() {
  uint8_t status = NRF_ReadStatus();

  if (status & STATUS_MASK_MAX_RT) {
    // Max retries while sending.
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_MAX_RT);
    com_ack = TRANSMIT_FAILED;
  }


  if (status & STATUS_MASK_TX_DS) {
    // ACK received
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_TX_DS);
    com_ack = TRANSMIT_OK;
  }

  if (status & STATUS_MASK_RX_DR) {
    // Received packet
    uint8_t pipe = (status & STATUS_MASK_RX_P_NO) >> 1;
    COM_RF_Receive(pipe);
  }
}

TransmitStatus COM_RF_Transmit(uint8_t robot, uint8_t* data, uint8_t len) {
  tx_semaphore_get(&semaphore, TX_WAIT_FOREVER);
  uint8_t addr[5] = ROBOT_ACTION_ADDR(robot);
  com_ack = TRANSMIT_ONGOING;
  data[0] |= msg_order[robot];
  msg_order[robot] += 16;
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5); // to receive auto-ACK
  int retries = 3;
  NRF_Transmit(data, len);
  while (retries > 0) {
    for (int i = 0; i < 100 && com_ack == TRANSMIT_ONGOING; ++i) {
      tx_thread_sleep(1);
    }
    if (com_ack != TRANSMIT_FAILED) {
      break;
    }
    com_ack = TRANSMIT_ONGOING;
    NRF_ReTransmit();
    --retries;
  }

  if (com_ack != TRANSMIT_OK) {
    NRF_SendCommand(NRF_CMD_FLUSH_TX);
  }

  NRF_EnterMode(NRF_MODE_RX);
  tx_semaphore_put(&semaphore);
  return com_ack;
}

void COM_RF_PrintInfo() {
  for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
    if (connected_robots[i] != ROBOT_DISCONNECTED) {
      LOG_INFO("Robot %d connected\r\n", i);
    } else {
      LOG_INFO("Robot %d not connected\r\n", i);
    }
  }
  NRF_PrintFIFOStatus();
  NRF_PrintStatus();
  NRF_PrintConfig();
}

void COM_RF_Receive(uint8_t pipe) {
  uint8_t len = 0;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);

  uint8_t payload[len];
  NRF_ReadPayload(payload, len);

  if (len > 2 && payload[0] == 0x57 && payload[1] == 0x75) {
    // We received a LOG_BASESTATION(...)
    LOG_INFO("ROBOT %d: %s\r\n", payload[2], payload + 3);
  }

  if (len == 5) {
    uint32_t magic = payload[0] << 24 | (payload[1] << 16) | (payload[2] << 8) | (payload[3]);
    uint8_t id = payload[4];

    if (magic == 0x4df84279 && id < MAX_ROBOT_COUNT) {
      // We received a ping
      if (connected_robots[id] != ROBOT_PENDING) {
        LOG_INFO("Robot %d connected\r\n", id);
      }
      connected_robots[id] = ROBOT_CONNECTED;
    } else {
      LOG_INFO("Received invalid packet: 0x%#08x\r\n", magic);
    }
  }

  NRF_SetRegisterBit(NRF_REG_STATUS, 6);
}

uint8_t* COM_CreateDummyPacket(uint8_t robot_id, uint8_t* len) {
  Command *cmd = malloc(sizeof(Command));

  command__init(cmd);

  cmd->command_id = ACTION_TYPE__MOVE_TO_ACTION;
  cmd->robot_id = robot_id;

  cmd->dest = malloc(sizeof(Vector3D));

  vector3_d__init(cmd->dest);
  cmd->dest->x = 1;
  cmd->dest->y = 2;
  cmd->dest->w = 3;

  *len = command__get_packed_size(cmd);
  uint8_t *buffer = malloc(*len);

  command__pack(cmd, buffer);
  free(cmd->dest);

  return buffer;
}

UINT COM_ParsePacket(NX_PACKET *packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch (packet_type) {
  case SSL_WRAPPER: {
    int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;
    ParsedFrame *parsed_frame = NULL;
    parsed_frame = parsed_frame__unpack(NULL, length, packet->nx_packet_prepend_ptr);
    if (parsed_frame == NULL) {
      ret = NX_INVALID_PACKET;
    } else {
      LOG_INFO("Received msg\r\n");

      free(parsed_frame);
    }
  } break;
  case ROBOT_COMMAND: {
    int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;

    if (length > 31) {
      LOG_ERROR("Robot command packet over 31 bytes (%d bytes)\r\n", length);
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
            //no_robot_responses[command->robot_id]++;
            //if (no_robot_responses[command->robot_id] > MAX_NO_RESPONSES) {
            //  connected_robots[command->robot_id] = ROBOT_DISCONNECTED;
            //  no_robot_responses[command->robot_id] = 0;
            //  LOG_INFO("Disconnected robot #%d after %d TX attempts\r\n", command->robot_id, MAX_NO_RESPONSES+1);
            //}
          } else {
            //no_robot_responses[command->robot_id] = 0;
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
