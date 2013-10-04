#ifndef __CPHIDGETRFID
#define __CPHIDGETRFID
#include "cphidget.h"

/** \defgroup phidrfid Phidget RFID 
 * \ingroup phidgets
 * Calls specific to the Phidget RFID. See the product manual for more specific API details, supported functionality, units, etc.
 * @{
 */

DPHANDLE(RFID)
CHDRSTANDARD(RFID)

/**
 * Gets the number of outputs supported by this board.
 * @param phid An attached phidget rfid handle.
 * @param count The output count.
 */
CHDRGET(RFID,OutputCount,int *count)
/**
 * Gets the state of an output.
 * @param phid An attached phidget rfid handle.
 * @param index The output index.
 * @param outputState The output state. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRGETINDEX(RFID,OutputState,int *outputState)
/**
 * Sets the state of an output.
 * @param phid An attached phidget rfid handle.
 * @param index The output index.
 * @param outputState The output state. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRSETINDEX(RFID,OutputState,int outputState)
/**
 * Set an output change handler. This is called when an output changes.
 * @param phid An attached phidget rfid handle.
 * @param fptr Callback function pointer.
 * @param userPtr A pointer for use by the user - this value is passed back into the callback function.
 */
CHDREVENTINDEX(RFID,OutputChange,int outputState)

/**
 * Gets the state of the antenna.
 * @param phid An attached phidget rfid handle.
 * @param antennaState The antenna state. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRGET(RFID,AntennaOn,int *antennaState)
/**
 * Sets the state of the antenna. Note that the antenna must be enabled before tags will be read.
 * @param phid An attached phidget rfid handle.
 * @param antennaState The antenna state. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRSET(RFID,AntennaOn,int antennaState)
/**
 * Gets the state of the onboard LED.
 * @param phid An attached phidget rfid handle.
 * @param LEDState The LED state. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRGET(RFID,LEDOn,int *LEDState)
/**
 * Sets the state of the onboard LED.
 * @param phid An attached phidget rfid handle.
 * @param LEDState The LED state. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRSET(RFID,LEDOn,int LEDState)

/**
 * Gets the last tag read by the reader. This tag may or may not still be on the reader.
 * @param phid An attached phidget rfid handle.
 * @param tag The tag. This must be an unsigned char array of size 5.
 */
CHDRGET(RFID,LastTag,unsigned char *tag)
/**
 * Gets the tag present status. This is whether or not a tag is being read by the reader.
 * @param phid An attached phidget rfid handle.
 * @param status The tag status. Possible values are \ref PTRUE and \ref PFALSE.
 */
CHDRGET(RFID,TagStatus,int *status)
/**
 * Set a tag handler. This is called when a tag is first detected by the reader.
 * @param phid An attached phidget rfid handle.
 * @param fptr Callback function pointer.
 * @param userPtr A pointer for use by the user - this value is passed back into the callback function.
 */
CHDREVENT(RFID,Tag,unsigned char *tag)
/**
 * Set a tag lost handler. This is called when a tag is no longer detected by the reader.
 * @param phid An attached phidget rfid handle.
 * @param fptr Callback function pointer.
 * @param userPtr A pointer for use by the user - this value is passed back into the callback function.
 */
CHDREVENT(RFID,TagLost,unsigned char *tag)

#ifndef REMOVE_DEPRECATED
DEP_CHDRGET("Deprecated - use CPhidgetRFID_getOutputCount",RFID,NumOutputs,int *)
#endif

#ifndef EXTERNALPROTO

#define RFID_PACKET_TAG 0
#define RFID_PACKET_OUTPUT_ECHO 1

#define RFID_LED_FLAG 0x04
#define RFID_ANTENNA_FLAG 0x08

#define RFID_MAXOUTPUTS 2

struct _CPhidgetRFID {
	CPhidget phid;

	int (CCONV *fptrOutputChange)(CPhidgetRFIDHandle, void *, int, int);
	int (CCONV *fptrTag)(CPhidgetRFIDHandle, void *, unsigned char *);
	int (CCONV *fptrTagLost)(CPhidgetRFIDHandle, void *, unsigned char *);

	void *fptrOutputChangeptr;
	void *fptrTagptr;
	void *fptrTagLostptr;

	// Values returned from the device
	unsigned char outputEchoState[RFID_MAXOUTPUTS];
	unsigned char antennaEchoState;
	unsigned char ledEchoState;
	
	unsigned char outputState[RFID_MAXOUTPUTS];
	unsigned char antennaState;
	unsigned char ledState;

	unsigned char lastTag[5];
	TIME	lastTagTime;
	unsigned char tagPresent;
	unsigned char tagEvent;

	unsigned char fullStateEcho;
	
	CThread tagTimerThread;

	unsigned char outputPacket[8];
	unsigned int outputPacketLen;
} typedef CPhidgetRFIDInfo;
#endif

/** @} */

#endif
