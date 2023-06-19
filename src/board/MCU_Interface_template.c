/*!*****************************************************************
 * \file    MCU_Interface_template.c
 * \brief   S2LP library low level functions.
 *******************************************************************/

#include "MCU_Interface_template.h"

#include "S2LP_Types.h"

/*** MCU INTERFACE TEMPLATE functions ***/

/*******************************************************************/
void SdkEvalSpiInit(void) {
	/* To be implemented by the device manufacturer */
}

/*******************************************************************/
void SdkEvalSpiDeinit(void) {
	/* To be implemented by the device manufacturer */
}

/*******************************************************************/
S2LPStatus SdkEvalSpiWriteRegisters(uint8_t register_address, uint8_t data_size, uint8_t* data) {
	/* To be implemented by the device manufacturer */
	S2LPStatus s2lp_status;
	return s2lp_status;
}

/*******************************************************************/
S2LPStatus SdkEvalSpiReadRegisters(uint8_t register_address, uint8_t data_size, uint8_t* data) {
	/* To be implemented by the device manufacturer */
	S2LPStatus s2lp_status;
	return s2lp_status;
}

/*******************************************************************/
S2LPStatus SdkEvalSpiCommandStrobes(uint8_t commmand) {
	/* To be implemented by the device manufacturer */
	S2LPStatus s2lp_status;
	return s2lp_status;
}

/*******************************************************************/
S2LPStatus SdkEvalSpiWriteFifo(uint8_t data_size, uint8_t* data) {
	/* To be implemented by the device manufacturer */
	S2LPStatus s2lp_status;
	return s2lp_status;
}

/*******************************************************************/
S2LPStatus SdkEvalSpiReadFifo(uint8_t data_size, uint8_t* data) {
	/* To be implemented by the device manufacturer */
	S2LPStatus s2lp_status;
	return s2lp_status;
}

/*******************************************************************/
void SdkEvalEnterShutdown(void) {
	/* To be implemented by the device manufacturer */
}

/*******************************************************************/
void SdkEvalExitShutdown(void) {
	/* To be implemented by the device manufacturer */
}

/*******************************************************************/
SFlagStatus SdkEvalCheckShutdown(void) {
	/* To be implemented by the device manufacturer */
    SFlagStatus shutdown_pin_state;
    return shutdown_pin_state;
}
