/*
 *	Authors:		Eduardo Nunes
 *					Geraldine Barreto
 *	Version:		1.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License)
 *	
 *	Desscription:	Communication protocol for the Motion Controller Unit on
*					LSS-based robots.
 *					Offers support for Arduino Uno, Mega and others through
 *					the use of the Stream class for serial communication.
 */

#ifndef LSS_MCU_H
#define LSS_MCU_H

#define MCU_SupportSoftwareSerial
// Uncomment the line below to disable software serial. This will prevent inclusion of the library, freeing some space.
// If you want to use WIFI control you will need to comment the line below.
#undef MCU_SupportSoftwareSerial

#define MCU_SupportPPM
// Uncomment the line below to disable PPM. This will prevent inclusion of the library, freeing some space.
// If you want to use RC control you will need to comment the line below.
//#undef MCU_SupportPPM

// Ensure compatibility
#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

// Other requirements
#include "LSS.h"
#include "string.h"
#ifdef MCU_SupportSoftwareSerial
#include "SoftwareSerial.h"
#endif

// Constants
#define MCU_SupportsSettingTimeouts
//> String processing
#ifndef LSS_H
#define IS_AF(c)					((c >= 'A') && (c <= 'F'))
#define IS_af(c)					((c >= 'a') && (c <= 'f'))
#define IS_09(c)					((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)				(IS_AF(c) || IS_af(c) || IS_09(c))
#define ISVALIDDEC(c)				(IS_09(c))
#define CONVERTDEC(c)				(c - '0')
#define CONVERTHEX_alpha(c)			(IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)				(IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))
#endif
//> Bus communication
#define MCU_DefaultBaud				(38400)

#define MCU_Timeout					100		// in ms
#define MCU_CommandStart			("#")
#define MCU_CommandReplyStart		("*")
#define MCU_CommandEnd				("\r")

//> MCU constants
#define MCU_ID_Default				(100)
#define MCU_ID_Min					(100)
#define MCU_ID_Max					(250)
#define MCU_Mode255ID				(255)
#define BroadcastID					(254)

enum MCU_LastCommStatus
{
	MCU_CommStatus_Idle,
	MCU_CommStatus_ReadSuccess,
	MCU_CommStatus_ReadTimeout,
	MCU_CommStatus_ReadWrongID,
	MCU_CommStatus_ReadWrongIdentifier,
	MCU_CommStatus_ReadWrongFormat,
	MCU_CommStatus_ReadNoBus,
	MCU_CommStatus_ReadUnknown,
	MCU_CommStatus_WriteSuccess,
	MCU_CommStatus_WriteNoBus,
	MCU_CommStatus_WriteUnknown
};

enum MCU_Status
{
	MCU_StatusUnknown,
	MCU_StatusLimp,
	MCU_StatusFreeMoving,
	MCU_StatusAccelerating,
	MCU_StatusTravelling,
	MCU_StatusDecelerating,
	MCU_StatusHolding,
	MCU_StatusOutsideLimits,
	MCU_StatusStuck,   //cannot move at current speed setting
	MCU_StatusBlocked, //same as stuck but reached maximum duty and still can't move
	MCU_StatusSafeMode,
	MCU_StatusLast
};
enum MCU_Model
{
	MCU_Model2IO,
	MCU_ModelBOT,
	MCU_ModelRP2040,
	MCU_ModelESP32,
	MCU_ModelUnknown
};
#define MCU_MODEL_2IO	"MCU-2IO"
#define MCU_MODEL_BOT	"MCU-BOT"
#define MCU_MODEL_RPI	"MCU-RPI"
#define MCU_MODEL_ESP	"MCU-ESP"

//> Parameter for query
enum MCU_QueryType
{
	MCU_QuerySession = 0,
	MCU_QueryConfig = 1,
	MCU_QueryInstantaneousSpeed = 2,
	MCU_QueryTargetTravelSpeed = 3
};

//> Parameter for query distance sensor
enum MCU_QueryTypeDistance
{
	MCU_Query_Sharp_GP2Y0A41SK0F = 1,
	MCU_Query_Sharp_GP2Y0A21YK0F = 2,
	MCU_Query_Sharp_GP2Y0A02YK0F = 3
};

//> Parameter for Serial/RC mode change
enum MCU_ConfigMode
{
	MCU_ModeSerial = 0,
	MCU_ModePositionRC = 1,
};

//> Commands - actions
#define MCU_ActionReset				("RESET")

//> Commands - queries
#define MCU_QueryStatus				("Q")
#define MCU_QuerySpeed				("QS")
#define MCU_QueryID					("QID")
#define MCU_QueryBaud				("QB")
#define MCU_QueryModelString		("QMS")
#define MCU_QueryAnalog				("QA")

//> Commands - configurations
#define MCU_ConfigID						("CID")
#define MCU_ConfigBaud						("CB")
#define MCU_ConfigSpeed						("CS")
#define MCU_ConfigModeRC					("CRC")

///////////////////////////////////////////////////


//>Commands - motion values
#define MCU_MotionValue 					("V")

enum Motion_commands{
	//>Commands -  motions
	MCU_Walking, 
	MCU_Rotation,
	MCU_Roll,
	MCU_Pitch,
	MCU_Yaw,
	MCU_FrontalOffset,
	MCU_Height,
	MCU_LateralOffset,
	MCU_GaitType,
	//>Commands - predefined sequences
	MCU_Up = 10, 
	MCU_Sit,
	MCU_Lay,
	MCU_Paw,
	MCU_Wiggle,
	MCU_Tinkle,
	MCU_Stretch,
	MCU_Sequence,
	MCU_Jog_On,
	MCU_Jog_Off
};

// library interface description
class MCU
{
public:
	// Public functions - Class
	static void setReadTimeouts(uint32_t start_response_timeout=MCU_Timeout, uint32_t msg_char_timeout=MCU_Timeout);
	static int timedRead(void);
	bool charToInt(char * inputstr, int32_t * intnum);
#ifdef MCU_SupportSoftwareSerial
	static void initBus(SoftwareSerial & s, uint32_t baud);
#endif
	static void initBus(HardwareSerial & s, uint32_t baud);
	static void closeBus(void);
	static bool genericWrite(uint8_t id, const char * cmd);
	static bool genericWrite(uint8_t id, const char * cmd, int16_t value);
	static bool genericWrite(uint8_t id, const char * cmd, int16_t value, const char * parameter, int16_t parameter_value);
	static void genericRead(int16_t * myArray);
	static int16_t valueRead(const char * value);
	static void motionRead(int16_t * myArray);
	static void lssRead(uint8_t id, int c);

	// Public attributes - Class

	// Public functions - Instance
	//> Constructors/destructor
	MCU();
	MCU(uint8_t id);
	~MCU(void);

	//> Get/set for private attributes
	static uint8_t getID(void);
	static void setID(uint8_t);
	MCU_LastCommStatus getLastCommStatus(void);

	//> Actions
	bool reset(void);

	//> Queries
	MCU_Status getStatus(void);
	uint8_t getSpeed(void);
	uint8_t getBaud(void);
	MCU_Model getModel(void);
	uint16_t getDistance(MCU_QueryTypeDistance queryTypeDistance);

	//> Configs
	// bool setSpeed(uint16_t value, MCU_SetType setType = MCU_SetSession);
	bool setMode(MCU_ConfigMode value);
	// Public attributes - Instance

private:
	// Private functions - Class

	// Private attributes - Class
	static bool hardwareSerial;
	static Stream * bus;
	static MCU_LastCommStatus lastCommStatus;
	static volatile unsigned int readID;
	static volatile unsigned int readVal;
	static char value[4];
	static char cmd[4];
	static uint32_t _msg_char_timeout;   // timeout waiting for characters inside of packet
	// Private functions - Instance

	// Private attributes - Instance
	static uint8_t mcuID;
};

#endif
