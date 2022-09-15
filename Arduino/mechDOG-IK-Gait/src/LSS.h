/*
 *	Author:			Sebastien Parent-Charette (support@robotshop.com)
 *	Version:		1.2.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
 *	
 *	Desscription:	A library that makes using the LSS simple.
 *					Offers support for both Arduino Uno, Mega and others through
 *					the use of the Stream class for communication.
 */

// Ensure this library description is only included once
#ifndef LSS_H
#define LSS_H

#define LSS_SupportSoftwareSerial
// Uncomment the line below to disable software serial. This will prevent inclusion of the library, freeing some space.
// By default, if you are not using SoftwareSerial anywhere, the compiler should remove it anyway.
#undef LSS_SupportSoftwareSerial

// Ensure compatibility
#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

// Other requirements
#include "string.h"
#ifdef LSS_SupportSoftwareSerial
#include "SoftwareSerial.h"
#endif

// Constants
#define LSS_SupportsSettingTimeouts
//> String processing
#define IS_AF(c)					((c >= 'A') && (c <= 'F'))
#define IS_af(c)					((c >= 'a') && (c <= 'f'))
#define IS_09(c)					((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)				(IS_AF(c) || IS_af(c) || IS_09(c))
#define ISVALIDDEC(c)				(IS_09(c))
#define CONVERTDEC(c)				(c - '0')
#define CONVERTHEX_alpha(c)			(IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)				(IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

//> Bus communication
#define LSS_DefaultBaud				(115200)
#define LSS_MaxTotalCommandLength	(30 + 1)	// ex: #999XXXX-2147483648\r; Adding 1 for end string char (\0)
// ex: #999XX000000000000000000\r;
#define LSS_Timeout					100		// in ms
#define LSS_CommandStart			("#")
#define LSS_CommandReplyStart		("*")
#define LSS_CommandEnd				("\r")
#define LSS_FirstPositionDisabled	("DIS")

//> Servo constants
#define LSS_ID_Default				(0)
#define LSS_ID_Min					(0)
#define LSS_ID_Max					(250)
#define LSS_Mode255ID				(255)
#define LSS_BroadcastID				(254)

enum LSS_LastCommStatus
{
	LSS_CommStatus_Idle,
	LSS_CommStatus_ReadSuccess,
	LSS_CommStatus_ReadTimeout,
	LSS_CommStatus_ReadWrongID,
	LSS_CommStatus_ReadWrongIdentifier,
	LSS_CommStatus_ReadWrongFormat,
	LSS_CommStatus_ReadNoBus,
	LSS_CommStatus_ReadUnknown,
	LSS_CommStatus_WriteSuccess,
	LSS_CommStatus_WriteNoBus,
	LSS_CommStatus_WriteUnknown
};

enum LSS_Status
{
	LSS_StatusUnknown,
	LSS_StatusLimp,
	LSS_StatusFreeMoving,
	LSS_StatusAccelerating,
	LSS_StatusTravelling,
	LSS_StatusDecelerating,
	LSS_StatusHolding,
	LSS_StatusOutsideLimits,
	LSS_StatusStuck,   //cannot move at current speed setting
	LSS_StatusBlocked, //same as stuck but reached maximum duty and still can't move
	LSS_StatusSafeMode,
	LSS_StatusLast
};
enum LSS_Model
{
	LSS_ModelHighTorque,
	LSS_ModelStandard,
	LSS_ModelHighSpeed,
	LSS_ModelUnknown
};
#define LSS_MODEL_HT1	"LSS-HT1"
#define LSS_MODEL_ST1	"LSS-ST1"
#define LSS_MODEL_HS1	"LSS-HS1"

//> Parameter for query
enum LSS_QueryType
{
	LSS_QuerySession = 0,
	LSS_QueryConfig = 1,
	LSS_QueryInstantaneousSpeed = 2,
	LSS_QueryTargetTravelSpeed = 3
};

//> Parameter for query distance sensor
enum LSS_QueryTypeDistance
{
	LSS_Query_Sharp_GP2Y0A41SK0F = 1,
	LSS_Query_Sharp_GP2Y0A21YK0F = 2,
	LSS_Query_Sharp_GP2Y0A02YK0F = 3
};

//> Parameter for setter
enum LSS_SetType
{
	LSS_SetSession = 0,
	LSS_SetConfig = 1
};

//> Parameter for Serial/RC mode change
enum LSS_ConfigMode
{
	LSS_ModeSerial = 0,
	LSS_ModePositionRC = 1,
	LSS_ModeWheelRC = 2
};

//> Parameter for gyre direction
enum LSS_ConfigGyre
{
	LSS_GyreClockwise = 1,
	LSS_GyreCounterClockwise = -1
};

//> LED colors
enum LSS_LED_Color
{
	LSS_LED_Black = 0,
	LSS_LED_Red = 1,
	LSS_LED_Green = 2,
	LSS_LED_Blue = 3,
	LSS_LED_Yellow = 4,
	LSS_LED_Cyan = 5,
	LSS_LED_Magenta = 6,
	LSS_LED_White = 7
};

//> Commands - actions
#define LSS_ActionReset				("RESET")
#define LSS_ActionLimp				("L")
#define LSS_ActionHold				("H")
#define LSS_ActionParameterTime		("T")
#define LSS_ActionParameterCurrentHold		("CH")
#define LSS_ActionParameterSpeed	("S")
#define LSS_ActionMove				("D")
#define LSS_ActionMoveRelative		("MD")
#define LSS_ActionWheel				("WD")
#define LSS_ActionWheelRPM			("WR")

//> Commands - actions (settings)
#define LSS_ActionOriginOffset		("O")
#define LSS_ActionAngularRange		("AR")
#define LSS_ActionMaxSpeed			("SD")
#define LSS_ActionMaxSpeedRPM		("SR")
#define LSS_ActionColorLED			("LED")
#define LSS_ActionGyreDirection		("G")

//> Commands - actions (advanced settings)
#define LSS_ActionAngularStiffness			("AS")
#define LSS_ActionAngularHoldingStiffness	("AH")
#define LSS_ActionAngularAcceleration		("AA")
#define LSS_ActionAngularDeceleration		("AD")
#define LSS_ActionEnableMotionControl		("EM")
#define LSS_FilterPositionCount				("FPC")

//> Commands - queries
#define LSS_QueryStatus				("Q")
#define LSS_QueryOriginOffset		("QO")
#define LSS_QueryAngularRange		("QAR")
#define LSS_QueryPositionPulse		("QP")
#define LSS_QueryPosition			("QD")
#define LSS_QuerySpeed				("QWD")
#define LSS_QuerySpeedRPM			("QWR")
#define LSS_QuerySpeedPulse			("QS")
#define LSS_QueryMaxSpeed			("QSD")
#define LSS_QueryMaxSpeedRPM		("QSR")
#define LSS_QueryColorLED			("QLED")
#define LSS_QueryGyre				("QG")
#define LSS_QueryID					("QID")
#define LSS_QueryBaud				("QB")
#define LSS_QueryFirstPosition		("QFD")
#define LSS_QueryModelString		("QMS")
#define LSS_QuerySerialNumber		("QN")
#define LSS_QueryFirmwareVersion	("QF")
#define LSS_QueryVoltage			("QV")
#define LSS_QueryTemperature		("QT")
#define LSS_QueryCurrent			("QC")
#define LSS_QueryAnalog				("QA")

//> Commands - queries (advanced)
#define LSS_QueryAngularStiffness			("QAS")
#define LSS_QueryAngularHoldingStiffness	("QAH")
#define LSS_QueryAngularAcceleration		("QAA")
#define LSS_QueryAngularDeceleration		("QAD")
#define LSS_QueryEnableMotionControl		("QEM")
#define LSS_QueryFilterPositionCount		("QFPC")
#define LSS_QueryBlinkingLED				("QLB")

//> Commands - configurations
#define LSS_ConfigID						("CID")
#define LSS_ConfigBaud						("CB")
#define LSS_ConfigOriginOffset				("CO")
#define LSS_ConfigAngularRange				("CAR")
#define LSS_ConfigMaxSpeed					("CSD")
#define LSS_ConfigMaxSpeedRPM				("CSR")
#define LSS_ConfigColorLED					("CLED")
#define LSS_ConfigGyreDirection				("CG")
#define LSS_ConfigFirstPosition				("CFD")
#define LSS_ConfigModeRC					("CRC")
#define LSS_ConfigFilterPositionCount		("CFPC")

//> Commands - configurations (advanced)
#define LSS_ConfigAngularStiffness			("CAS")
#define LSS_ConfigAngularHoldingStiffness	("CAH")
#define LSS_ConfigAngularAcceleration		("CAA")
#define LSS_ConfigAngularDeceleration		("CAD")
#define LSS_ConfigBlinkingLED				("CLB")

// library interface description
class LSS
{
public:
	// Public functions - Class
	static void setReadTimeouts(uint32_t start_response_timeout=LSS_Timeout, uint32_t msg_char_timeout=LSS_Timeout);
	static int timedRead(void);
	bool charToInt(char * inputstr, int32_t * intnum);
	//static void initBus(Stream &, uint32_t);
#ifdef LSS_SupportSoftwareSerial
	static void initBus(SoftwareSerial & s, uint32_t baud);
#endif
	static void initBus(HardwareSerial & s, uint32_t baud);
	static void closeBus(void);
	static bool genericWrite(uint8_t id, const char * cmd);
	static bool genericWrite(uint8_t id, const char * cmd, int16_t value);
	static bool genericWrite(uint8_t id, const char * cmd, int16_t value, const char * parameter, int16_t parameter_value);
	static int16_t genericRead_Blocking_s16(uint8_t id, const char * cmd);
	static char * genericRead_Blocking_str(uint8_t id, const char * cmd);

	// Public attributes - Class

	// Public functions - Instance
	//> Constructors/destructor
	LSS();
	LSS (uint8_t);
	~LSS(void);

	//> Get/set for private attributes
	uint8_t getServoID(void);
	void setServoID(uint8_t);
	LSS_LastCommStatus getLastCommStatus(void);

	//> Actions
	bool reset(void);
	bool limp(void);
	bool hold(void);
	bool move(int16_t value);
	bool moveT(int16_t value, int16_t t_value);
	bool moveCH(int16_t value, int16_t ch_value);
	bool moveRelative(int16_t value);
	bool moveRelativeT(int16_t value, int16_t t_value);
	bool wheel(int16_t value);
	bool wheelRPM(int8_t value);

	//> Queries
	// uint8_t getID(void);
	// uint8_t getBaud(void);
	LSS_Status getStatus(void);
	int16_t getOriginOffset(LSS_QueryType queryType = LSS_QuerySession);
	uint16_t getAngularRange(LSS_QueryType queryType = LSS_QuerySession);
	uint16_t getPositionPulse(void);
	int32_t getPosition(void);
	int16_t getSpeed(void);
	int8_t getSpeedRPM(void);
	int8_t getSpeedPulse(void);
	uint16_t getMaxSpeed(LSS_QueryType queryType = LSS_QuerySession);
	int8_t getMaxSpeedRPM(LSS_QueryType queryType = LSS_QuerySession);
	LSS_LED_Color getColorLED(LSS_QueryType queryType = LSS_QuerySession);
	LSS_ConfigGyre getGyre(LSS_QueryType queryType = LSS_QuerySession);
	int16_t getFirstPosition(void);
	bool getIsFirstPositionEnabled(void);
	LSS_Model getModel(void);
	char * getSerialNumber(void);
	uint16_t getFirmwareVersion(void);
	uint16_t getVoltage(void);
	uint16_t getTemperature(void);
	uint16_t getCurrent(void);
	uint16_t getAnalog(void);
	uint16_t getDistance_mm(LSS_QueryTypeDistance queryTypeDistance);

	//> Queries (advanced)
	int8_t getAngularStiffness(LSS_QueryType queryType = LSS_QuerySession);
	int8_t getAngularHoldingStiffness(LSS_QueryType queryType = LSS_QuerySession);
	int16_t getAngularAcceleration(LSS_QueryType queryType = LSS_QuerySession);
	int16_t getAngularDeceleration(LSS_QueryType queryType = LSS_QuerySession);
	bool getIsMotionControlEnabled(void);
	int16_t getFilterPositionCount(LSS_QueryType queryType = LSS_QuerySession);
	uint8_t getBlinkingLED(void);

	//> Configs
	bool setOriginOffset(int16_t value, LSS_SetType setType = LSS_SetSession);
	bool setAngularRange(uint16_t value, LSS_SetType setType = LSS_SetSession);
	bool setMaxSpeed(uint16_t value, LSS_SetType setType = LSS_SetSession);
	bool setMaxSpeedRPM(int8_t value, LSS_SetType setType = LSS_SetSession);
	bool setColorLED(LSS_LED_Color value, LSS_SetType setType = LSS_SetSession);
	bool setGyre(LSS_ConfigGyre value, LSS_SetType setType = LSS_SetSession);
	bool setFirstPosition(int16_t value);
	bool clearFirstPosition(void);
	bool setMode(LSS_ConfigMode value);

	//> Configs (advanced)
	bool setAngularStiffness(int8_t value, LSS_SetType setType = LSS_SetSession);
	bool setAngularHoldingStiffness(int8_t value, LSS_SetType setType = LSS_SetSession);
	bool setAngularAcceleration(int16_t value, LSS_SetType setType = LSS_SetSession);
	bool setAngularDeceleration(int16_t value, LSS_SetType setType = LSS_SetSession);
	bool setMotionControlEnabled(bool value);
	bool setFilterPositionCount(int16_t value, LSS_SetType setType = LSS_SetSession);
	bool setBlinkingLED(uint8_t value);

	// Public attributes - Instance

private:
	// Private functions - Class

	// Private attributes - Class
	static bool hardwareSerial;
	static Stream * bus;
	static LSS_LastCommStatus lastCommStatus;
	static volatile unsigned int readID;
	static char value[24];
	static uint32_t _msg_char_timeout;   // timeout waiting for characters inside of packet
	// Private functions - Instance

	// Private attributes - Instance
	uint8_t servoID = LSS_ID_Default;
};

#endif
