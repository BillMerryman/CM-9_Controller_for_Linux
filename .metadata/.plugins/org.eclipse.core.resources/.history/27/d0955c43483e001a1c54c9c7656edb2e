/** @file motionManager.cpp
 *  @brief Functions for managing motions.
 *
 *  These are the functions that: setup the pointers used to
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

#include <cstring>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "motionManager.hpp"

int configurePort(int fd)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) return -1;

	cfsetospeed(&tty, USB_SERIAL_BAUD_RATE);
	cfsetispeed(&tty, USB_SERIAL_BAUD_RATE);

	tty.c_iflag &= ~IGNBRK;         				// disable break processing
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); 		// shut off xon/xoff ctrl

	tty.c_lflag = 0;                				// no signaling chars, no echo, no canonical processing

	tty.c_oflag = 0;                				// no remapping, no delays

	tty.c_cc[VMIN]  = 0;            				// read doesn't block
	tty.c_cc[VTIME] = 10;            				// 1 second read timeout

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_cflag |= (CLOCAL | CREAD);				// ignore modem controls, enable reading
	tty.c_cflag &= ~(PARENB | PARODD);  			// no parity
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
	return 0;
}

void transferFlashToMotionPage(int fd, uint8_t pageNumber)
{
	uint8_t requestParameterBuffer[] = { pageNumber };
	uint8_t error = 0;
	transmitPacket(fd, CM5_ID, INST_TRANSFER_FLASH_TO_MOTION_PAGE, &error, requestParameterBuffer, sizeof(requestParameterBuffer), NULL);
}

void executeMotionPage(int fd, uint8_t pageNumber)
{
	uint8_t requestParameterBuffer[] = { pageNumber };
	uint8_t error = 0;
	transmitPacket(fd, CM5_ID, INST_EXECUTE_MOTION_PAGE, &error, requestParameterBuffer, sizeof(requestParameterBuffer), NULL);
}


//change this to exchangePackets
bool transmitPacket(int fd, uint8_t id, uint8_t instruction, uint8_t *error, uint8_t *requestBuffer, uint8_t requestBufferLength, uint8_t *responseBuffer)
{
	if (sendPacket(fd, id, instruction, requestBuffer, requestBufferLength))
	{
		if (receivePacket(fd, error, responseBuffer))
		{
			return true;
		}
	}
	return false;
}

bool sendPacket(int fd, uint8_t id, uint8_t instruction, uint8_t *parameters, uint8_t parametersLength)
{
	//go through and have functions check the return value
	//also either have the error output write to the output window, or
	//expand the response fields. Or maybe just put what we did get back in the response fields
	//and pop up an 'error' modal dialog alert. Maybe zero out the arrays before we
	//populate and send them
	uint8_t outputBuffer[BUFFER_SIZE];
	uint8_t readPosition = 0;
	uint8_t writePosition = 0;

	assemblePacket(outputBuffer, &readPosition, &writePosition, id, instruction, parameters, parametersLength);
	if(write(fd, outputBuffer, writePosition)==-1) return false;
	return true;
}

bool receivePacket(int fd, uint8_t *error, uint8_t *parameters)
{
	uint8_t charBuffer[1];
	uint8_t RS485RxBuffer[BUFFER_SIZE];
	uint8_t RS485RxReadPosition = 0;
	uint8_t RS485RxWritePosition = 0;
	uint8_t returnedPacketLength = 0;
	uint8_t returnedID = 0;

	while (true)
	{
		if(read(fd, charBuffer, 1) == 0) return false;
		RS485RxBuffer[RS485RxWritePosition++] = charBuffer[0];
		if (!isPacketReady(RS485RxBuffer, &RS485RxReadPosition, &RS485RxWritePosition, &returnedPacketLength)) continue;
		disassemblePacket(RS485RxBuffer, &RS485RxReadPosition, &RS485RxWritePosition, &returnedID, error, parameters, returnedPacketLength);
		return true;
	}
}

void assemblePacket(uint8_t *buffer, uint8_t *readPosition, uint8_t *writePosition, uint8_t ID, uint8_t instructionOrError, uint8_t *TxParameters, uint8_t TxParameterLength)
{

	while(getAvailableMessageBuffer(readPosition, writePosition) < (HEADER_WIDTH + INSTRUCTION_OR_ERROR_WIDTH + TxParameterLength + CHECKSUM_WIDTH + 1));

	buffer[*writePosition] = 0xFF;
	(*writePosition)++;
	buffer[*writePosition] = 0xFF;
	(*writePosition)++;
	buffer[*writePosition] = ID;
	(*writePosition)++;
	uint8_t payloadLength = ERROR_WIDTH + TxParameterLength + CHECKSUM_WIDTH;
	buffer[*writePosition] = payloadLength;
	(*writePosition)++;
	buffer[*writePosition] = instructionOrError;
	(*writePosition)++;

	uint8_t checkSum = ID + payloadLength + instructionOrError;

	for(uint8_t count = 0; count < TxParameterLength; count++)
	{
		buffer[*writePosition] = TxParameters[count];
		checkSum += TxParameters[count];
		(*writePosition)++;
	}

	buffer[*writePosition] = ~checkSum;
	(*writePosition)++;

}

bool isPacketReady(uint8_t *buffer, uint8_t *readPosition, uint8_t *writePosition, uint8_t *parametersLength)
{

	uint8_t occupiedBufferSize = getOccupiedMessageBuffer(readPosition, writePosition);

	//if we don't even have the sentinel yet, exit
	if(occupiedBufferSize < PACKET_FLAG_WIDTH) return false;

	//if we have what should at least be a sentinel, but it isn't,
	//consume the buffer until we find one, or exit if it is exhausted
	while(buffer[*readPosition] != 0xFF || buffer[(uint8_t)((*readPosition) + 1)] != 0xFF)
	{
		(*readPosition)++;
		occupiedBufferSize = getOccupiedMessageBuffer(readPosition, writePosition);
		if(occupiedBufferSize < PACKET_FLAG_WIDTH) return false;
	}

	//if we don't have a headers worth yet, exit
	if(occupiedBufferSize < HEADER_WIDTH) return false;

	uint8_t parameterSizePosition = *readPosition + PACKET_FLAG_WIDTH + ID_WIDTH;
	uint8_t packetSize = HEADER_WIDTH + buffer[parameterSizePosition];

	//if we don't have what the header says is a packets worth, exit
	if(occupiedBufferSize < packetSize) return false;

	uint8_t checkSumPosition = (*readPosition + packetSize) - CHECKSUM_WIDTH;
	uint8_t checkSum = 0;
	uint8_t tmpReadPosition = *readPosition + PACKET_FLAG_WIDTH;
	while(tmpReadPosition != checkSumPosition) checkSum += buffer[tmpReadPosition++];
	checkSum = ~checkSum;
	if(checkSum != buffer[tmpReadPosition++])
	{
		*readPosition = tmpReadPosition;
		return false;
	}

	*parametersLength = buffer[parameterSizePosition] - (INSTRUCTION_OR_ERROR_WIDTH + CHECKSUM_WIDTH);
	return true;

}

void disassemblePacket(uint8_t *buffer, uint8_t *readPosition, uint8_t *writePosition, uint8_t *ID, uint8_t *instructionOrError, uint8_t *parameters, uint8_t parametersLength)
{

	*readPosition += PACKET_FLAG_WIDTH;
	(*ID) = buffer[*readPosition];
	(*readPosition)++;
	uint8_t packetLength = buffer[*readPosition];
	(*readPosition)++;
	(*instructionOrError) = buffer[*readPosition];
	(*readPosition)++;

	uint8_t tmpParametersLength = packetLength - (INSTRUCTION_OR_ERROR_WIDTH + CHECKSUM_WIDTH);
	for(uint8_t parametersPosition = 0; parametersPosition < parametersLength; parametersPosition++)
	{
		parameters[parametersPosition] = buffer[*readPosition];
		(*readPosition)++;
	}
	(*readPosition)++;

}

uint8_t getOccupiedMessageBuffer(uint8_t *readPosition, uint8_t *writePosition)
{
	//This might be a sloppy way of doing this. We are counting on the case
	//where the read and write positions are the same to return zero by
	//virtue of the fact that it is returning 8 bits. When read and write
	//are the same, their difference is zero, zero is subtracted from the
	//total buffer size, and so returns 256. But 256 doesn't fit in 8
	//bits, so the value is truncated to the lower 8 bits, which are zero,
	//so the value is zero. The behavior is correct, how we are getting it
	//is not clearly reflected in the logic...
	//(this is better (*readPosition <= *writePosition))
	return (*readPosition < *writePosition) ? (*writePosition - *readPosition) : (BUFFER_SIZE - (*readPosition - *writePosition));

}

uint16_t getAvailableMessageBuffer(uint8_t *readPosition, uint8_t *writePosition)
{

	return BUFFER_SIZE - getOccupiedMessageBuffer(readPosition, writePosition);

}




