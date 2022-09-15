/*
 *	Author:			Sebastien Parent-Charette (support@robotshop.com)
 *	Version:		1.2.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License version 3)
 *	
 *	Desscription:	A library that makes using the LSS simple.
 *					Offers support for Arduino Uno, Mega and others through
 *					the use of the Stream class for serial communication.
 */

// Required main library
#include "LSS.h"

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Class attributes instantiation   ---- ---- ---- ---- ---- ---- ---- ---- ----
//> Bus & status related
bool LSS::hardwareSerial;
Stream * LSS::bus;
LSS_LastCommStatus LSS::lastCommStatus = LSS_CommStatus_Idle;
uint32_t LSS::_msg_char_timeout = LSS_Timeout;

//> Command reading/writing
volatile unsigned int LSS::readID;  // sscanf - assumes this
char LSS::value[24];

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Constructor  ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

// Useful for declaring arrays; defaults to ID=0
LSS::LSS()
{
	// Init state
	this->servoID = LSS_ID_Default;
}

// Recommended constructor
LSS::LSS(uint8_t id)
{
	// Init state
	this->servoID = id;
}

LSS::~LSS(void)
{
}

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Public functions (class)    ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
void LSS::setReadTimeouts(uint32_t start_response_timeout, uint32_t msg_char_timeout)
{
	_msg_char_timeout = msg_char_timeout;
	bus->setTimeout(start_response_timeout);
}


int LSS::timedRead(void)
{
	int c;
	unsigned long startMillis = millis();
	do
	{
		c = LSS::bus->read();
		if (c >= 0)
			return (c);
	} while (millis() - startMillis < _msg_char_timeout);
	return (-1);     // -1 indicates timeout
}

bool LSS::charToInt(char * inputstr, int32_t * intnum)
{
	uint8_t neg = 0, i = 0, j = 0;  //, res = 0;
	int32_t val = 0;
	bool err;

	if (!inputstr) return true;

	if (inputstr[0] == '0' && (inputstr[1] == 'x' || inputstr[1] == 'X'))
	{
		if (inputstr[2] == '\0') //"0x" only
		{
			val = 0;
			*intnum = 0;
			err = false;
		}
		else
		{
			for (i = 2; i < 11; i++)
			{
				if (inputstr[i] == '\0')
				{
					*intnum = val;
					break;
				}
				if (ISVALIDHEX(inputstr[i]))
				{
					val = (val << 4) + CONVERTHEX(inputstr[i]);
					err = true;
				}
				else
				{
					err = false;
					break;
				}
			}
		}
		/* over 8 digit hex --invalid */
		if (i >= 11)
		{
			err = false;
		}
	}
	else /* max 10-digit decimal input */
	{
		if (inputstr[i] == '-')
		{
			// Indicate a negative int32_t
			neg = 1;

			// Skip the first character
			j = 1;
		}
		for (i = j; i < 11; i++)
		{
			if (inputstr[i] == '\0')
			{
				if (neg)
				{
					if (i == j)
					{
						err = false;
					}
					else
					{
						val = -val;
					}
				}
				else
				{
					if (i == 0)
					{
						err = false;
					}
				}

				*intnum = val;

				break;
			}
			else if (ISVALIDDEC(inputstr[i]))
			{
				val = val * 10 + CONVERTDEC(inputstr[i]);
				err = true;
			}
			else
			{
				err = false;
				break;
			}
		}
		/* Over 10 digit decimal --invalid */
		if (i >= 11)
		{
			err = false;
		}
	}

	// Success
	return (err);
}

#ifdef LSS_SupportSoftwareSerial
// Initialize bus using a software serial
void LSS::initBus(SoftwareSerial &s, uint32_t baud)
{
	bus = &s;
	bus->setTimeout(LSS_Timeout);
	hardwareSerial = false;
	s.begin(baud);
	s.listen();
}
#endif

// Initialize bus using a hardware serial
void LSS::initBus(HardwareSerial &s, uint32_t baud)
{
	bus = &s;
	bus->setTimeout(LSS_Timeout);
	hardwareSerial = true;
	s.begin(baud);
}

// Close the bus (stream), free pins and null reference
void LSS::closeBus(void)
{
#ifdef LSS_SupportSoftwareSerial
	if (hardwareSerial)
	{
		static_cast<HardwareSerial*>(bus)->end();
	}
	else
	{
		static_cast<SoftwareSerial*>(bus)->end();
	}
	bus = (Stream*) nullptr;
#else
	static_cast<HardwareSerial*>(bus)->end();
#endif
}

// Build & write a LSS command to the bus using the provided ID (no value)
// Max size for cmd = (LSS_MaxTotalCommandLength - 1)
bool LSS::genericWrite(uint8_t id, const char * cmd)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	// Build command
	bus->write('#');
	// Servo ID
	bus->print(id, DEC);
	// Command
	bus->write(cmd);
	// Command end
	bus->write('\r');
	// Success
	lastCommStatus = LSS_CommStatus_WriteSuccess;
	return (true);
}

// Build & write a LSS command to the bus using the provided ID and value
// Max size for cmd = (LSS_MaxTotalCommandLength - 1)
bool LSS::genericWrite(uint8_t id, const char * cmd, int16_t value)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	bus->write('#');
	// Servo ID
	bus->print(id, DEC);
	// Command
	bus->write(cmd);
	// Value
	bus->print(value, DEC);
	// Command end
	bus->write('\r');
	// Success
	lastCommStatus = LSS_CommStatus_WriteSuccess;
	return (true);
}

// Build & write a LSS command to the bus using the provided ID and value
// Max size for cmd = (LSS_MaxTotalCommandLength - 1)
bool LSS::genericWrite(uint8_t id, const char * cmd, int16_t value, const char * parameter, int16_t parameter_value)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	bus->write('#');
	// Servo ID
	bus->print(id, DEC);
	// Command
	bus->print(cmd);
	// Value
	bus->print(value, DEC);
	bus->write(parameter);
	// Parameter Value
	bus->print(parameter_value, DEC);
	// Command end
	bus->write('\r');
	// Success
	lastCommStatus = LSS_CommStatus_WriteSuccess;
	return (true);
}
//==============================================================================
char * LSS::genericRead_Blocking_str(uint8_t id, const char * cmd)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_ReadNoBus;
		return ((char *) nullptr);
	}

	// 

	// Read from bus until first character; exit if not found before timeout
	if (!(bus->find(LSS_CommandReplyStart)))
	{
		lastCommStatus = LSS_CommStatus_ReadTimeout;
		return ((char *) nullptr);
	}

	// Ok we have the * now now lets get the servo ID from the message.
	readID = 0;
	int c;
	bool valid_field = 0;
	while ((c = LSS::timedRead()) >= 0)
	{
		if ((c < '0') || (c > '9')) break;	// not a number character
		readID = readID * 10 + c - '0';
		valid_field = true;
	}

	if ((!valid_field) || (readID != id))
	{
		lastCommStatus = LSS_CommStatus_ReadWrongID;
		// BUGBUG Should we clear out until CR?
		return ((char *) nullptr);
	} 

	// Now lets validate the right CMD
	for (;;)
	{
		if (c != *cmd)
		{
			lastCommStatus = LSS_CommStatus_ReadWrongIdentifier;
			return ((char *) nullptr);			
		}
		cmd++;
		if (*cmd == '\0')
			break;
		c = LSS::timedRead();
	}
	size_t maxLength = (LSS_MaxTotalCommandLength - 1);
	size_t index = 0;


	while (index < maxLength)
	{
		c = LSS::timedRead();
		if (c < 0 || c == LSS_CommandEnd[0])
			break;
		value[index] = (char) c;
		index++;
	}
	value[index] = '\0';

	// Return value (success)
	if (c < 0) 
	{
		// did not get the ending CR
		lastCommStatus = LSS_CommStatus_ReadTimeout;
		return ((char *) nullptr);
	}

	lastCommStatus = LSS_CommStatus_ReadSuccess;
	return value;
}

//==============================================================================
int16_t LSS::genericRead_Blocking_s16(uint8_t id, const char * cmd)
{
	// Let the string function do all of the main parsing work.
	char *valueStr = genericRead_Blocking_str(id, cmd);
	if (valueStr == (char *) nullptr)
	{
		// the above method will have already set the error condition. 
		return 0;
	}

	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = LSS_CommStatus_ReadNoBus;
		return (0);
	}

	// convert the value string to value
	int16_t value = 0;
	int16_t value_sign = 1;
	for(;;)
	{
		if ((*valueStr >= '0') && (*valueStr <= '9'))
			value = value * 10 + *valueStr - '0';
		else if (*valueStr == '-')
			value_sign = -1;
		else 
			break;
		valueStr++; 
	}
	// now see if we exited with valid number
	if (*valueStr != '\0')
	{
		lastCommStatus = LSS_CommStatus_ReadWrongID;
		return (0);
	}
	// return the computed value
	return value * value_sign; 
} 


// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Public functions (instance) ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

// Get current servo ID
uint8_t LSS::getServoID(void)
{
	return (this->servoID);
}

// Set current servo ID (guards for safe range of [LSS_ID_Min, LSS_ID_Max])
void LSS::setServoID(uint8_t id)
{
	// LSS_ID_Min is 0 so this one does nothing
	//if (id < LSS_ID_Min)
	//	id = 0;
	if (id > LSS_ID_Max)
		id = LSS_ID_Max;
	this->servoID = id;
}

LSS_LastCommStatus LSS::getLastCommStatus(void)
{
	return (LSS::lastCommStatus);
}

// --- Actions ---

// Reset the LSS
// Note: no waiting is done here. LSS will take a bit more than a second to reset/start responding to commands.
bool LSS::reset(void)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionReset));
}

// Make LSS limp
bool LSS::limp(void)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionLimp));
}

// Make LSS hold current position
bool LSS::hold(void)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionHold));
}

// Make LSS move to specified position in 1/10°
bool LSS::move(int16_t value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionMove, value));
}

// Make LSS move to specified position in 1/10° with T parameter
bool LSS::moveT(int16_t value, int16_t t_value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionMove, value, LSS_ActionParameterTime, t_value));
}

// Make LSS move to specified position in 1/10° with CH parameter
bool LSS::moveCH(int16_t value, int16_t ch_value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionMove, value, LSS_ActionParameterCurrentHold, ch_value));
}

// Perform relative move by specified amount of 1/10°
bool LSS::moveRelative(int16_t value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionMoveRelative, value));
}

// Perform relative move by specified amount of 1/10° with T parameter
bool LSS::moveRelativeT(int16_t value, int16_t t_value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionMoveRelative, value, LSS_ActionParameterTime, t_value));
}

// Make LSS rotate at set speed in (1/10°)/s
bool LSS::wheel(int16_t value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionWheel, value));
}

// Make LSS rotate at set speed in RPM
bool LSS::wheelRPM(int8_t value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionWheelRPM, value));
}

//> Queries
/*
 // Not implemented since they serve no purpose; the IDs and baud are fixed by design
 uint8_t getID(void){}
 uint8_t getBaud(void){}
 */
// Returns current status
LSS_Status LSS::getStatus(void)
{
	// Variables
	LSS_Status value = LSS_StatusUnknown;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryStatus)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (LSS_Status) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryStatus);

	// Return result
	return (value);
}

// Returns origin offset in 1/10°
int16_t LSS::getOriginOffset(LSS_QueryType queryType)
{
	// Variables
	int16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryOriginOffset, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryOriginOffset);

	// Return result
	return (value);
}

// Returns angular range in 1/10°
uint16_t LSS::getAngularRange(LSS_QueryType queryType)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAngularRange, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAngularRange);

	// Return result
	return (value);
}

// Returns position in µs pulses (RC style)
uint16_t LSS::getPositionPulse(void)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryPositionPulse)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryPositionPulse);

	// Return result
	return (value);
}

// Returns position in 1/10°
int32_t LSS::getPosition(void)
{
	// Variables
	char * valueStr;
	int32_t valuePos = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryPosition)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (0);
	}

	// Read response from servo (as string)
	valueStr = LSS::genericRead_Blocking_str(this->servoID, LSS_QueryPosition);

	// Check for disabled first position
	if (LSS::charToInt(valueStr, &valuePos))
	{
		return (valuePos);
	}
	else
	{
		// Unable to convert valueStr to int16_t
		return (0);
	}

	// Return result
	return (0);	// should not be reachable
}

// Returns speed in (1/10°)/s
int16_t LSS::getSpeed(void)
{
	// Variables
	int16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QuerySpeed)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QuerySpeed);

	// Return result
	return (value);
}

int8_t LSS::getSpeedRPM(void)
{
	// Variables
	int8_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QuerySpeedRPM)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int8_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QuerySpeedRPM);

	// Return result
	return (value);
}

int8_t LSS::getSpeedPulse(void)
{
	// Variables
	int8_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QuerySpeedPulse)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int8_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QuerySpeedPulse);

	// Return result
	return (value);
}

uint16_t LSS::getMaxSpeed(LSS_QueryType queryType)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryMaxSpeed, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryMaxSpeed);

	// Return result
	return (value);
}

int8_t LSS::getMaxSpeedRPM(LSS_QueryType queryType)
{
	// Variables
	int8_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryMaxSpeedRPM, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int8_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryMaxSpeedRPM);

	// Return result
	return (value);
}

LSS_LED_Color LSS::getColorLED(LSS_QueryType queryType)
{
	// Variables
	LSS_LED_Color value = LSS_LED_Black;  // was 0 but removed warning
 
	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryColorLED, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (LSS_LED_Color) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryColorLED);

	// Return result
	return (value);
}

LSS_ConfigGyre LSS::getGyre(LSS_QueryType queryType)
{
	// Variables
	LSS_ConfigGyre value = (LSS_ConfigGyre)0;  // Note Not a valid value for this. 

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryGyre, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (LSS_ConfigGyre) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryGyre);

	// Return result
	return (value);
}

int16_t LSS::getFirstPosition(void)
{
	// Variables
	char * valueStr;
	int32_t valuePosLarger = 0;
	int16_t valuePos = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryFirstPosition)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (0);
	}

	// Read response from servo (as string)
	valueStr = LSS::genericRead_Blocking_str(this->servoID, LSS_QueryFirstPosition);

	// Check for disabled first position
	if (strcmp(valueStr, LSS_FirstPositionDisabled) == 0)
	{
		// First position is not defined; return 0
		return (0);
	}
	else
	{
		if (LSS::charToInt(valueStr, &valuePosLarger))
		{
			valuePos = (int16_t) valuePosLarger;
		}
		else
		{
			// Unable to convert valueStr to int16_t
			return (0);
		}
		return (valuePos);
	}

	// Return result
	return (0);	// should not be reachable
}

bool LSS::getIsFirstPositionEnabled(void)
{
	// Variables
	char * valueStr;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryFirstPosition)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (false);
	}

	// Read response from servo (as string)
	valueStr = LSS::genericRead_Blocking_str(this->servoID, LSS_QueryFirstPosition);
	if (lastCommStatus == LSS_CommStatus_ReadSuccess)
	{
		// Check if first position is disabled
		return (strcmp(valueStr, LSS_FirstPositionDisabled));
	}
	else
	{
		// Read was not completed;
		return (false);
	}
}

LSS_Model LSS::getModel(void)
{
	// Variables
	char * valueStr;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryModelString)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (LSS_ModelUnknown);
	}

	// Read response from servo
	valueStr = LSS::genericRead_Blocking_str(this->servoID, LSS_QueryModelString);

	if (strcmp(valueStr, LSS_MODEL_HT1) == 0)
	{
		return (LSS_ModelHighTorque);
	}
	else if (strcmp(valueStr, LSS_MODEL_ST1) == 0)
	{
		return (LSS_ModelStandard);
	}
	else if (strcmp(valueStr, LSS_MODEL_HS1) == 0)
	{
		return (LSS_ModelHighSpeed);
	}
	else
	{
		return (LSS_ModelUnknown);
	}
}

char * LSS::getSerialNumber(void)
{
	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QuerySerialNumber)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (nullptr);
	}

	// Read response from servo
	return (LSS::genericRead_Blocking_str(this->servoID, LSS_QuerySerialNumber));
}

uint16_t LSS::getFirmwareVersion(void)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryFirmwareVersion)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryFirmwareVersion);

	// Return result
	return (value);
}

uint16_t LSS::getVoltage(void)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryVoltage)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryVoltage);

	// Return result
	return (value);
}

uint16_t LSS::getTemperature(void)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryTemperature)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryTemperature);

	// Return result
	return (value);
}

uint16_t LSS::getCurrent(void)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryCurrent)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryCurrent);

	// Return result
	return (value);
}

//> Queries (advanced)
int8_t LSS::getAngularStiffness(LSS_QueryType queryType)
{
	// Variables
	int8_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAngularStiffness, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int8_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAngularStiffness);

	// Return result
	return (value);
}

int8_t LSS::getAngularHoldingStiffness(LSS_QueryType queryType)
{
	// Variables
	int8_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAngularHoldingStiffness, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int8_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAngularHoldingStiffness);

	// Return result
	return (value);
}

int16_t LSS::getAngularAcceleration(LSS_QueryType queryType)
{
	// Variables
	int16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAngularAcceleration, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAngularAcceleration);

	// Return result
	return (value);
}

int16_t LSS::getAngularDeceleration(LSS_QueryType queryType)
{
	// Variables
	int16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAngularDeceleration, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAngularDeceleration);

	// Return result
	return (value);
}

bool LSS::getIsMotionControlEnabled(void)
{
	// Variables
	bool value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryEnableMotionControl)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (bool) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryEnableMotionControl);

	// Return result
	return (value);
}

int16_t LSS::getFilterPositionCount(LSS_QueryType queryType) {
	// Variables
	int16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryFilterPositionCount, queryType)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (int16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryFilterPositionCount);

	// Return result
	return (value);
	
}

uint8_t LSS::getBlinkingLED(void)
{
	// Variables
	uint8_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryBlinkingLED)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint8_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryBlinkingLED);

	// Return result
	return (value);
}

uint16_t LSS::getAnalog(void)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAnalog)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAnalog);

	// Return result
	return (value);
}

uint16_t LSS::getDistance_mm(LSS_QueryTypeDistance queryTypeDistance)
{
	// Variables
	uint16_t value = 0;

	// Ask servo for status; exit if it failed
	if (!(LSS::genericWrite(this->servoID, LSS_QueryAnalog, queryTypeDistance)))
	{
		LSS::lastCommStatus = LSS_CommStatus_WriteNoBus;
		return (value);
	}

	// Read response from servo
	value = (uint16_t) LSS::genericRead_Blocking_s16(this->servoID, LSS_QueryAnalog);

	// Return result
	return (value);
}

//> Configs
bool LSS::setOriginOffset(int16_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionOriginOffset, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigOriginOffset, value));
			break;
		}
	}

	// Return result
	return (value);
}

bool LSS::setAngularRange(uint16_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionAngularRange, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigAngularRange, value));
			break;
		}
	}
	return false;
}

bool LSS::setMaxSpeed(uint16_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionMaxSpeed, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigMaxSpeed, value));
			break;
		}
	}
	return false;
}

bool LSS::setMaxSpeedRPM(int8_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionMaxSpeedRPM, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigMaxSpeedRPM, value));
			break;
		}
	}
	return false;
}

bool LSS::setColorLED(LSS_LED_Color value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionColorLED, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigColorLED, value));
			break;
		}
	}
	return false;
}

bool LSS::setGyre(LSS_ConfigGyre value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionGyreDirection, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigGyreDirection, value));
			break;
		}
	}
	return false;
}

bool LSS::setFirstPosition(int16_t value)
{
	return (LSS::genericWrite(this->servoID, LSS_ConfigFirstPosition, value));
}

bool LSS::clearFirstPosition(void)
{
	return (LSS::genericWrite(this->servoID, LSS_ConfigFirstPosition));
}

bool LSS::setMode(LSS_ConfigMode value)
{
	return (LSS::genericWrite(this->servoID, LSS_ConfigModeRC, value));
}

//> Configs (advanced)
bool LSS::setAngularStiffness(int8_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionAngularStiffness, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigAngularStiffness, value));
			break;
		}
	}
	return false;
}

bool LSS::setAngularHoldingStiffness(int8_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionAngularHoldingStiffness, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigAngularHoldingStiffness, value));
			break;
		}
	}
	return false;
}

bool LSS::setAngularAcceleration(int16_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionAngularAcceleration, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigAngularAcceleration, value));
			break;
		}
	}
	return false;
}

bool LSS::setAngularDeceleration(int16_t value, LSS_SetType setType)
{
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_ActionAngularDeceleration, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigAngularDeceleration, value));
			break;
		}
	}
	return false;
}

bool LSS::setMotionControlEnabled(bool value)
{
	return (LSS::genericWrite(this->servoID, LSS_ActionEnableMotionControl, value));
}

bool LSS::setFilterPositionCount(int16_t value, LSS_SetType setType)
{
	
	switch (setType)
	{
		case (LSS_SetSession):
		{
			return (LSS::genericWrite(this->servoID, LSS_FilterPositionCount, value));
			break;
		}
		case (LSS_SetConfig):
		{
			return (LSS::genericWrite(this->servoID, LSS_ConfigFilterPositionCount, value));
			break;
		}
	}
	return false;
}


bool LSS::setBlinkingLED(uint8_t value)
{
	return (LSS::genericWrite(this->servoID, LSS_ConfigBlinkingLED, value));
}

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Private functions (class)   ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

/*
 static void LSS::?????(void)
 {
 }
 */

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Private functions (instance)     ---- ---- ---- ---- ---- ---- ---- ---- ----
/*
 void LSS::?????(void)
 {
 }
 */
