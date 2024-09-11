#ifndef COM_H
#define COM_H

/* Public includes */
#include <stdbool.h>
#include "main.h"
#include <radio.h>

typedef enum TransmitStatus {
    TRANSMIT_OK, TRANSMIT_ONGOING, TRANSMIT_FAILED
} TransmitStatus;

typedef enum RobotConnection {
    ROBOT_CONNECTED, ROBOT_DISCONNECTED, ROBOT_PENDING
} RobotConnection;

/* Public function declarations */

/**
 * Configures the NRF device according to this robots serial number.
 *
 * @param hspi The handle for the SPI communication.
 */
void COM_RF_Init(SPI_HandleTypeDef* hspi);

/**
 * Transmit a data buffer to a robot.
 * The first data byte should be msg id < 16.
 *
 * @param robot id of destination robot.
 * @param data data to send, data[0] = msg id
 * @param len length of data, <= 32
 * @return status
 */
TransmitStatus COM_RF_Transmit(uint8_t robot, uint8_t* data, uint8_t len);

/**
 * Broadcast a data buffer to all robots.
 * The first data byte should be msg id.
 *
 * @param data data to send, data [0] = msg id
 * @param len length of data, <= 32
 * @return status
 */
TransmitStatus COM_RF_Broadcast(uint8_t* data, uint8_t len);

/**
 * Parse the received message and handle it correctly.
 *
 * @param pipe What pipe the message was received on.
 */
void COM_RF_Receive(uint8_t pipe);

/**
 * Handles interrupts sent from the IRQ pin on the NRF.
 */
void COM_RF_HandleIRQ(void);

/**
 * Printf:s status and FIFO status registers from the NRF.
 */
void COM_RF_PrintInfo(void);

extern volatile RobotConnection connected_robots[MAX_ROBOT_COUNT];

/**
 * Send a ping message to robots. If ping_all is false,
 * only connected robots will be pinged.
 */
void COM_RF_PingRobots(bool ping_all);

#endif /* COM_H */
