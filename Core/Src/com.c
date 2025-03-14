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
#define MAX_NO_RESPONSES 200

/* Private structs */
typedef enum RobotComStatus {
    COMSTAT_INIT,
    COMSTAT_CONNECT,
    COMSTAT_OK,
    COMSTAT_FAIL,
    COMSTAT_DISCONNECTED,
    COMSTAT_INVALID_PACKET,
} RobotComStatus;

/* Private variables */
TX_SEMAPHORE semaphore;
volatile TransmitStatus com_ack; // Status of acknowledgement of a transmission
static LOG_Module internal_log_mod;
uint8_t msg_order[MAX_ROBOT_COUNT];

/*
 * Public functions implementations
 */

void COM_RF_Init(SPI_HandleTypeDef* hspi) {
  LOG_InitModule(&internal_log_mod, "RF", LOG_LEVEL_TRACE, 0);

  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    LOG_WARNING("Couldn't verify nRF24 SPI...\r\n");
    Error_Handler();
  }

  if (tx_semaphore_create(&semaphore, "NRF-semaphore", 1) != TX_SUCCESS) {
    LOG_ERROR("Failed creating NRF-semaphore\r\n");
  }

  for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
    last_robot_com_status[i] = COMSTAT_INIT;
    msg_order[i] = 0;
  }

  com_ack = TRANSMIT_INIT;

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // See nRF24L01+ chapter 6.3 for more...
  // Set the RF channel frequency to 2500, i.e. outside of wifi range
  // It's defined as: 2400 + NRF_REG_RF_CH [MHz]
  // NRF_REG_RF_CH can 0-127, but not all values seem to work.
  // 2525 and below works, 2527 had issues...
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x7d); // 2525
  //NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x64); // 2500

  // Setup the TX address.
  // We also have to set pipe 0 to receive on the same address.
  uint8_t tx_addr[5] = ROBOT_ACTION_ADDR(0);
  NRF_WriteRegister(NRF_REG_TX_ADDR, tx_addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, tx_addr, 5);

  uint8_t rx_addr[5] = CONTROLLER_ADDR;
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, rx_addr, 5);

  // Disable retry transmissions
  NRF_WriteRegisterByte(NRF_REG_SETUP_RETR, 0x00);

  // Enter receive mode
  NRF_EnterMode(NRF_MODE_RX);
  LOG_INFO("Initialized...\r\n");
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

void COM_RF_Transmit(uint8_t robot, uint8_t* data, uint8_t len) {
  tx_semaphore_get(&semaphore, TX_WAIT_FOREVER);

  uint8_t addr[5] = ROBOT_ACTION_ADDR(robot);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5); // to receive auto-ACK
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_Transmit(data, len);
  NRF_SendCommand(NRF_CMD_FLUSH_TX);

  tx_semaphore_put(&semaphore);
}

void COM_RF_PrintInfo() {
  uint8_t ret = NRF_ReadStatus();

  if (!ret) {
    LOG_INFO("nRF24 not running...\r\n");
    return;
  }

  LOG_INFO("Status register: %02X\r\n", ret);
  LOG_INFO("TX_FULL:  %1X\r\n", ret & (1<<0));
  LOG_INFO("RX_P_NO:  %1X\r\n", (ret & (0x3<<1)) >> 1);
  LOG_INFO("MAX_RT:   %1X\r\n", (ret & (1<<4))    >> 4);
  LOG_INFO("TX_DS:    %1X\r\n", (ret & (1<<5))     >> 5);
  LOG_INFO("RX_DR:    %1X\r\n", (ret & (1<<6))     >> 6);
  LOG_INFO("\r\n");

  ret = NRF_ReadRegisterByte(NRF_REG_FIFO_STATUS);
  LOG_INFO("FIFO status register: %02X\r\n", ret);
  LOG_INFO("RX_EMPTY:   %2X\r\n", ret &  (1<<0));
  LOG_INFO("RX_FULL:    %2X\r\n", (ret & (1<<1)) >> 1);
  LOG_INFO("TX_EMPTY:   %2X\r\n", (ret & (1<<4)) >> 4);
  LOG_INFO("TX_FULL:    %2X\r\n", (ret & (1<<5)) >> 5);
  LOG_INFO("TX_REUSE:   %2X\r\n", (ret & (1<<6)) >> 6);
  LOG_INFO("\r\n");

  ret = NRF_ReadRegisterByte(NRF_REG_CONFIG);
  LOG_INFO("Config register: %02X\r\n", ret);
  LOG_INFO("PRIM_RX:      %1X\r\n", ret & (1<<0));
  LOG_INFO("PWR_UP:       %1X\r\n", ret & (1<<1));
  LOG_INFO("CRCO:         %1X\r\n", ret & (1<<2));
  LOG_INFO("EN_CRC:       %1X\r\n", ret & (1<<3));
  LOG_INFO("MASK_MAX_RT:  %1X\r\n", ret & (1<<4));
  LOG_INFO("MASK_TX_DS:   %1X\r\n", ret & (1<<5));
  LOG_INFO("MASK_RX_DR:   %1X\r\n", ret & (1<<6));
  LOG_INFO("\r\n");

  for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
    if (last_robot_com_status[i] == COMSTAT_OK) {
      LOG_INFO("Robot %d connected\r\n", i);
    }
  }

  LOG_INFO("\r\n");
}

void COM_RF_Receive(uint8_t pipe) {
  uint8_t len = 0;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);

  uint8_t payload[len];
  NRF_ReadPayload(payload, len);

  if (len > 2 && payload[0] == 0x57 && payload[1] == 0x75) {
    // We received a LOG_BASESTATION(...)
    LOG_INFO("ROBOT %d: %s\r\n", payload[2], payload + 3);
  } else {
    LOG_INFO("Received unknown RF package\r\n");
  }

  NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_TX_DS);
}

UINT COM_ParsePacket(NX_PACKET *packet, PACKET_TYPE packet_type) {
  UINT ret = NX_SUCCESS;

  switch (packet_type) {
  case ROBOT_COMMAND: {
    int length = packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr;

    if (length > 32) {
      LOG_ERROR("Robot command packet over 32 bytes (%d bytes)\r\n", length);
      ret = NX_INVALID_PACKET;
      return ret;
    }

    Command *command = NULL;
    command = command__unpack(NULL, length, packet->nx_packet_prepend_ptr);
    if (command == NULL) {
      LOG_ERROR("Invalid ethernet packet\r\n");
      return NX_INVALID_PACKET;
    }

    if (last_robot_com_status[command->robot_id] == COMSTAT_DISCONNECTED) {
      no_robot_responses[command->robot_id]++;
      if (no_robot_responses[command->robot_id] == MAX_NO_RESPONSES) {
        LOG_INFO("Robot %d still disconnected\r\n");
        no_robot_responses[command->robot_id] = 0;
      }
      return ret;
    }

    const ProtobufCEnumValue* enum_value = protobuf_c_enum_descriptor_get_value(&action_type__descriptor, command->command_id);

    uint8_t data[32];
    data[0] = 1;
    memcpy(data + 1, packet->nx_packet_prepend_ptr, length);

    TransmitStatus status = COM_RF_Transmit(command->robot_id, data, length + 1);
    if (status != TRANSMIT_OK) {
      if (last_robot_com_status[command->robot_id] != COMSTAT_FAIL) {
        LOG_INFO("Fail send robot #%d command %s\r\n", command->robot_id, enum_value->name);
        last_robot_com_status[command->robot_id] = COMSTAT_FAIL;
      }

      no_robot_responses[command->robot_id]++;

      if (no_robot_responses[command->robot_id] > MAX_NO_RESPONSES) {
        last_robot_com_status[command->robot_id] = COMSTAT_DISCONNECTED;
        no_robot_responses[command->robot_id] = 0;
        LOG_INFO("Disconnected robot #%d after %d TX attempts\r\n", command->robot_id, MAX_NO_RESPONSES+1);
      }
    } else {
      no_robot_responses[command->robot_id] = 0;

      if (last_robot_com_status[command->robot_id] != COMSTAT_OK) {
        LOG_INFO("Sent robot #%d command %s\r\n", command->robot_id, enum_value->name);
        last_robot_com_status[command->robot_id] = COMSTAT_OK;
      }
    }

    protobuf_c_message_free_unpacked(&command->base, NULL);
  } break;
  default:
    LOG_INFO("Unknown packet type: %d\r\n", packet_type);
    break;
  }

  if (ret != NX_SUCCESS) {
    LOG_WARNING("Failed to parse UDP packet\r\n");
  }

  return ret;
}
