#include "com.h"

/* Private includes */
#include <nrf24l01.h>
#include <nrf_helper_defines.h>
#include <string.h>
#include <tx_api.h>
#include <log.h>

/* Private defines */
#define PIPE_CONTROLLER 0
#define PIPE_VISION     1

#define CONNECT_MAGIC 0x4d, 0xf8, 0x42, 0x79

/* Private functions declarations */
//...

/*
 * Public functions implementations
 */


TX_SEMAPHORE semaphore;
// Status of acknowledgement of a transmission
volatile TransmitStatus com_ack;

static LOG_Module internal_log_mod;

void COM_Init(SPI_HandleTypeDef* hspi) {
  LOG_InitModule(&internal_log_mod, "RF", LOG_LEVEL_INFO);
  uint8_t address[5] = CONTROLLER_ADDR;
  com_ack = TRANSMIT_OK;
  if (tx_semaphore_create(&semaphore, "NRF-semaphore", 1) != TX_SUCCESS) {
    LOG_ERROR("Failed creating NRF-semaphore\r\n");
  }
  // Mark all robots as disconnected
  for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
    connected_robots[i] = ROBOT_DISCONNECTED;
  }

  NRF_Init(hspi, NRF_CSN_GPIO_Port, NRF_CSN_Pin, NRF_CE_GPIO_Port, NRF_CE_Pin);
  if(NRF_VerifySPI() != NRF_OK) {
    LOG_WARNING("Couldn't verify nRF24 SPI...\r\n");
  }

  // Resets all registers but keeps the device in standby-I mode
  NRF_Reset();

  // Set the RF channel frequency, it's defined as: 2400 + NRF_REG_RF_CH [MHz]
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x0F);

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
  /*
  uint8_t addr[5] = ROBOT_ACTION_ADDR(0);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5);
  uint8_t msg[] = {'P', 'i', 'n','g'};
  NRF_Transmit(msg, 4);*/

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
    connected_robots[i] = ROBOT_PENDING;
    uint8_t addr[5] = ROBOT_ACTION_ADDR(i);
    if (COM_RF_Transmit(addr, msg, 5) != TRANSMIT_OK) {
      LOG_INFO("Robot %d did not respond\r\n", i);
      continue;
    }
    for (int j = 0; j < 50; ++j) {
      tx_thread_sleep(1);
      if (connected_robots[i] == ROBOT_CONNECTED) {
        LOG_INFO("Robot %d responded\r\n", i);
        goto next;
      }
    }
    // Coming here means ack was received, but the robot still did not respond.
    LOG_WARNING("Robot %d has disconnected\r\n", i);

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

TransmitStatus COM_RF_Transmit(uint8_t* addr, uint8_t* data, uint8_t len) {
  tx_semaphore_get(&semaphore, TX_WAIT_FOREVER);
  com_ack = TRANSMIT_ONGOING;
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5);
  NRF_Transmit(data, len);
  while (com_ack == TRANSMIT_ONGOING) {
    tx_thread_sleep(1);
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


volatile RobotConnection connected_robots[MAX_ROBOT_COUNT];
/*
 * Private function implementations
 */

void COM_RF_Receive(uint8_t pipe) {
  uint8_t len = 0;
  NRF_SendReadCommand(NRF_CMD_R_RX_PL_WID, &len, 1);

  uint8_t payload[len];
  NRF_ReadPayload(payload, len);

  LOG_INFO("Payload of length %i on pipe %d\r\n", len, pipe);

  if (len == 5) {
    uint32_t magic = payload[0] << 24 | (payload[1] << 16) | (payload[2] << 8) | (payload[3]);
    uint8_t id = payload[4];
    if (magic == 0x4df84279 && id < MAX_ROBOT_COUNT) {
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