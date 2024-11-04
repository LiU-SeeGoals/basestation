#include "com.h"

/* Private includes */
#include <nrf24l01.h>
#include <nrf_helper_defines.h>
#include <tx_api.h>
#include <log.h>

/* Private defines */

/* Private functions declarations */
//...

/*
 * Public functions implementations
 */


TX_SEMAPHORE semaphore;
// Status of acknowledgement of a transmission
volatile TransmitStatus com_ack;
// Ack payload
volatile uint8_t ackpayload_len;
volatile uint8_t ackpayload[16];

static LOG_Module internal_log_mod;

uint8_t msg_order[MAX_ROBOT_COUNT];

void COM_RF_Init(SPI_HandleTypeDef* hspi) {
  LOG_InitModule(&internal_log_mod, "RF", LOG_LEVEL_INFO);
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

  // Set the RF channel frequency, it's defined as: 2400 + NRF_REG_RF_CH [MHz]
  NRF_WriteRegisterByte(NRF_REG_RF_CH, 0x0F);

  // Setup the TX address.
  // We also have to set pipe 0 to receive on the same address.
  uint8_t send_addr[5] = ROBOT_ADDR(ROBOT_ID_BROADCAST);

  NRF_WriteRegister(NRF_REG_TX_ADDR, send_addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, send_addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, address, 5);

  // We enable ACK payloads which needs dynamic payload to function.
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_ACK_PAY);
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_DPL);
  NRF_SetRegisterBit(NRF_REG_FEATURE, FEATURE_EN_DYN_ACK);
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

static uint8_t ping_msg[] = {MESSAGE_ID_PING, 'P', 'i', 'n', 'g'};

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
      if (COM_RF_Transmit(i, ping_msg, 5) != TRANSMIT_OK) {
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
    // ACK received (or no ack is used)
    NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_TX_DS);
    if (com_ack == TRANSMIT_ONGOING) {
      com_ack = TRANSMIT_OK;
    }
  }

  if (status & STATUS_MASK_RX_DR) {
    // Received packet
    uint8_t pipe = (status & STATUS_MASK_RX_P_NO) >> 1;
    COM_RF_Receive(pipe);
  }
}

TransmitStatus COM_RF_Transmit(uint8_t robot, uint8_t* data, uint8_t len) {
  uint8_t addr[5] = ROBOT_ADDR(robot);
  tx_semaphore_get(&semaphore, TX_WAIT_FOREVER);
  com_ack = TRANSMIT_ONGOING;
  data[0] |= msg_order[robot];
  msg_order[robot] += 16;
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5);
  NRF_Transmit(data, len);

  uint32_t now = HAL_GetTick();
  while (com_ack == TRANSMIT_ONGOING) {
    if (HAL_GetTick() - now > 3) {
      break;
    }
  }

  if (com_ack != TRANSMIT_OK) {
    // Flush tx buffer
    NRF_SendCommand(NRF_CMD_FLUSH_TX);
  }

  NRF_EnterMode(NRF_MODE_RX);
  tx_semaphore_put(&semaphore);
  return com_ack;
}

TransmitStatus COM_RF_TransmitWithResponse(uint8_t robot, uint8_t *data,
                                           uint8_t len, uint8_t *rx_buf,
                                           uint8_t* rx_len) {
  uint8_t addr[5] = ROBOT_ADDR(robot);
  tx_semaphore_get(&semaphore, TX_WAIT_FOREVER);
  com_ack = TRANSMIT_RESPONSE_PENDING;
  data[0] |= msg_order[robot];
  msg_order[robot] += 16;
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5);
  NRF_Transmit(data, len);

  uint32_t now = HAL_GetTick();
  while (com_ack == TRANSMIT_RESPONSE_PENDING) {
    if (HAL_GetTick() - now > 3) {
      break;
    }
  }

  if (com_ack != TRANSMIT_OK) {
    // Flush tx buffer
    NRF_SendCommand(NRF_CMD_FLUSH_TX);
  } else {
    *rx_len = ackpayload_len;
    for (int i = 0; i < *rx_len; ++i) {
      rx_buf[i] = ackpayload[i];
    }
  }

  NRF_EnterMode(NRF_MODE_RX);
  tx_semaphore_put(&semaphore);
  return com_ack;
}

TransmitStatus COM_RF_Broadcast(uint8_t *data, uint8_t len) {
  uint8_t addr[5] = ROBOT_ADDR(ROBOT_ID_BROADCAST);
  tx_semaphore_get(&semaphore, TX_WAIT_FOREVER);

  com_ack = TRANSMIT_ONGOING;
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_WriteRegister(NRF_REG_TX_ADDR, addr, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, addr, 5);
  NRF_TransmitNoAck(data, len);

  uint32_t now = HAL_GetTick();
  while (com_ack == TRANSMIT_ONGOING) {
    if (HAL_GetTick() - now > 3) {
      break;
    }
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
  NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_RX_DR);

  LOG_INFO("Payload of length %i on pipe %d\r\n", len, pipe);
  if (len == 5) {
    // Ping can happen spontaneously, event when waiting for response.
    uint32_t magic = CONNECT_MAGIC_READ(payload);
    uint8_t id = payload[4];
    if (magic == CONNECT_MAGIC && id < MAX_ROBOT_COUNT) {
      if (connected_robots[id] != ROBOT_PENDING) {
        LOG_INFO("Robot %d connected\r\n", id);
      }
      connected_robots[id] = ROBOT_CONNECTED;
      return;
    }
  }

  if (com_ack == TRANSMIT_RESPONSE_PENDING) {
    com_ack = TRANSMIT_OK;
    if (len < 16) {
      for (int i = 0; i < len; ++i) {
        ackpayload[i] = payload[i];
      }
      ackpayload_len = len;
    } else {
      // TRANSMIT_OK since transmit whent well.
      // Caller has to handle that no valid payload was received.
      ackpayload_len = 0;
    }
  }
}
