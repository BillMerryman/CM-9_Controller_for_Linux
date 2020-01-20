/** @file motionManager.hpp
 *  @brief Function prototypes for managing motions.
 *
 *  These are the prototypes for functions that: setup the pointers used to
 *  communicate with the memory shared between the application processor and PRU,
 *  place requests in the shared memory to the PRU motion worker, and to load a
 *  motion file from the file system into the shared memory so the PRU motion worker
 *  can retrieve and use them.
 *
 *  @author Bill Merryman
 *  @bug No known bugs.
 *
 *  Created on: Dec 9, 2019
 *
 */

#ifndef MOTIONMANAGER_HPP_
#define MOTIONMANAGER_HPP_

#define USB_SERIAL_BAUD_RATE						115200
#define USB_SERIAL_STOP_BITS						0x00
#define USB_SERIAL_PARITY							0x00
#define USB_SERIAL_NUMBER_OF_BITS					0x08

#define BUFFER_SIZE									256

#define PACKET_FLAG_WIDTH							2
#define ID_WIDTH									1
#define ID_POSITION									PACKET_FLAG_WIDTH
#define PAYLOAD_LENGTH_WIDTH						1
#define PAYLOAD_LENGTH_POSITION						ID_POSITION + ID_WIDTH
#define INSTRUCTION_OR_ERROR_WIDTH					1
#define INSTRUCTION_OR_ERROR_POSITION				PAYLOAD_LENGTH_POSITION + PAYLOAD_LENGTH_WIDTH
#define ERROR_WIDTH									1
#define ERROR_POSITION								PAYLOAD_LENGTH_POSITION + PAYLOAD_LENGTH_WIDTH
#define PARAMETER_START_POSITION					INSTRUCTION_OR_ERROR_POSITION + INSTRUCTION_OR_ERROR_WIDTH
#define CHECKSUM_WIDTH								1
#define HEADER_WIDTH								PACKET_FLAG_WIDTH + ID_WIDTH + PAYLOAD_LENGTH_WIDTH
#define NON_PARAMETER_WIDTH							HEADER_WIDTH + INSTRUCTION_OR_ERROR_WIDTH + CHECKSUM_WIDTH

#define CM5_ID										200

enum PROTOCOL
{
	//Robotis existing instructions
	INST_PING = 0x01,
	INST_READ_DATA = 0x02,
	INST_WRITE_DATA = 0x03,
	INST_REG_WRITE = 0x04,
	INST_ACTION = 0x05,
	INST_RESET = 0x06,
	INST_DIGITAL_RESET = 0x07,
	INST_SYSTEM_READ = 0x0C,
	INST_SYSTEM_WRITE = 0x0D,
	INST_SYNC_WRITE = 0x83,
	INST_SYNC_REG_WRITE = 0x84,
	//Added instructions
	//controller level
	INST_WRITE_PAGE_SECTION_TO_BUFFER = 0xA0,
	INST_READ_PAGE_SECTION_FROM_BUFFER = 0xA1,
	INST_TRANSFER_PAGE_BUFFER_TO_FLASH = 0xA2,
	INST_TRANSFER_PAGE_BUFFER_TO_MOTION_PAGE = 0xA3,
	INST_TRANSFER_FLASH_TO_PAGE_BUFFER = 0xA4,
	INST_TRANSFER_FLASH_TO_MOTION_PAGE = 0xA5,
	INST_EXECUTE_MOTION_PAGE = 0xA6,
	INST_BREAK_MOTION_PAGE = 0xA7,
	INST_STOP_MOTION_PAGE = 0xA8,
	INST_READ_AX12_IMAGE_IN_MEMORY = 0xB0,
	INST_WRITE_AX12_IMAGE_IN_MEMORY = 0xB1,
	INST_UPDATE_AX12_IMAGE_IN_MEMORY_FROM_DEVICE = 0xB2,
	INST_UPDATE_AX12_DEVICE_FROM_IMAGE_IN_MEMORY = 0xB3,
	INST_UPDATE_ALL_AX12_IMAGES_IN_MEMORY_FROM_DEVICES = 0xB4,
	INST_UPDATE_ALL_AX12_DEVICES_FROM_IMAGES_IN_MEMORY = 0xB5,
	INST_READ_AXS1_IMAGE_IN_MEMORY = 0xB8,
	INST_WRITE_AXS1_IMAGE_IN_MEMORY = 0xB9,
	INST_UPDATE_AXS1_IMAGE_IN_MEMORY_FROM_DEVICE = 0xBA,
	INST_UPDATE_AXS1_DEVICE_FROM_IMAGE_IN_MEMORY = 0xBB,
	INST_UPDATE_ALL_AXS1_IMAGES_IN_MEMORY_FROM_DEVICES = 0xBC,
	INST_UPDATE_ALL_AXS1_DEVICES_FROM_IMAGES_IN_MEMORY = 0xBD
};

int configurePort(int fd);
void transferFlashToMotionPage(int fd, uint8_t pageNumber);
void executeMotionPage(int fd, uint8_t pageNumber);
bool transmitPacket(int fd, uint8_t id, uint8_t instruction, uint8_t *error, uint8_t *requestBuffer, uint8_t requestBufferLength, uint8_t *responseBuffer);
bool sendPacket(int fd, uint8_t id, uint8_t instruction, uint8_t *parameters, uint8_t parametersLength);
bool receivePacket(int fd, uint8_t *error, uint8_t *parameters);
void assemblePacket(uint8_t *buffer, uint8_t *readPosition, uint8_t *writePosition, uint8_t ID, uint8_t instructionOrError, uint8_t *TxParameters, uint8_t TxParameterLength);
bool isPacketReady(uint8_t *buffer, uint8_t *readPosition, uint8_t *writePosition, uint8_t *parametersLength);
void disassemblePacket(uint8_t *buffer, uint8_t *readPosition, uint8_t *writePosition, uint8_t *ID, uint8_t *instructionOrError, uint8_t *parameters, uint8_t parametersLength);
uint8_t getOccupiedMessageBuffer(uint8_t *readPosition, uint8_t *writePosition);
uint16_t getAvailableMessageBuffer(uint8_t *readPosition, uint8_t *writePosition);

#endif /* MOTIONMANAGER_HPP_ */
