#ifndef COM_H
#define COM_H

/* Public includes */
#include "main.h"

#define MAX_ROBOT_COUNT 16

#define CONTROLLER_ADDR {2, 255, 255, 255, 255}
#define ROBOT_DATA_ADDR(id) {1, 255, 255, id, 128}
#define ROBOT_ACTION_ADDR(id) {1, 255, 255, id, 255}
#define ROBOT_PING_ADDR(id) {1, 255, 255, id, 0}

/* Public function declarations */

/**
 * Configures the NRF device according to this robots serial number.
 *
 * @param hspi The handle for the SPI communication.
 */
void COM_Init(SPI_HandleTypeDef* hspi);

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

extern uint8_t connected_robots[MAX_ROBOT_COUNT];

#endif /* COM_H */
