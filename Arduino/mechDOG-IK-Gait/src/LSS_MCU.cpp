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

#include "LSS_MCU.h"

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Class attributes instantiation   ---- ---- ---- ---- ---- ---- ---- ---- ----
//> Bus & status related
bool MCU::hardwareSerial;
Stream * MCU::bus;
MCU_LastCommStatus MCU::lastCommStatus = MCU_CommStatus_Idle;
uint32_t MCU::_msg_char_timeout = MCU_Timeout;

//> Command reading/writing
volatile unsigned int MCU::readID;  // sscanf - assumes this
volatile unsigned int MCU::readVal;
char MCU::value[4];
char MCU::cmd[4];
uint8_t MCU::mcuID;
// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Constructor  ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

// Useful for declaring arrays; defaults to ID=0
MCU::MCU()
{
	// Init state
	mcuID = MCU_ID_Default;
}

// Recommended constructor
MCU::MCU(uint8_t id)
{
	// Init state
	mcuID = id;
}

MCU::~MCU(void)
{
}

// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Public functions (class)    ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
void MCU::setReadTimeouts(uint32_t start_response_timeout, uint32_t msg_char_timeout)
{
	_msg_char_timeout = msg_char_timeout;
	bus->setTimeout(start_response_timeout);
}

int MCU::timedRead(void)
{
	int c;
	unsigned long startMillis = millis();
	do
	{
		c = MCU::bus->read();
		if (c >= 0)
			return (c);
	} while (millis() - startMillis < _msg_char_timeout);
	return (-1);     // -1 indicates timeout
}

bool MCU::charToInt(char * inputstr, int32_t * intnum)
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

#ifdef MCU_SupportSoftwareSerial
// Initialize bus using a software serial
void MCU::initBus(SoftwareSerial &s, uint32_t baud)
{
	bus = &s;
	bus->setTimeout(MCU_Timeout);
	hardwareSerial = false;
	s.begin(baud);
	s.listen();
	mcuID = MCU_ID_Default;
}
#endif

// Initialize bus using a hardware serial
void MCU::initBus(HardwareSerial &s, uint32_t baud)
{
	bus = &s;
	bus->setTimeout(MCU_Timeout);
	hardwareSerial = true;
	s.begin(baud);
	mcuID = MCU_ID_Default;
}

// Close the bus (stream), free pins and null reference
void MCU::closeBus(void)
{
#ifdef MCU_SupportSoftwareSerial
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

// Build & write a command to the bus using the provided ID (no value)
// Max size for cmd = (MCU_MaxTotalCommandLength - 1)
bool MCU::genericWrite(uint8_t id, const char * cmd)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_WriteNoBus;
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
	lastCommStatus = MCU_CommStatus_WriteSuccess;
	return (true);
}

// Build & write a command to the bus using the provided ID and value
// Max size for cmd = (MCU_MaxTotalCommandLength - 1)
bool MCU::genericWrite(uint8_t id, const char * cmd, int16_t value)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_WriteNoBus;
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
	lastCommStatus = MCU_CommStatus_WriteSuccess;
	return (true);
}

// Build & write a command to the bus using the provided ID and value
// Max size for cmd = (MCU_MaxTotalCommandLength - 1)
bool MCU::genericWrite(uint8_t id, const char * cmd, int16_t value, const char * parameter, int16_t parameter_value)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_WriteNoBus;
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
	lastCommStatus = MCU_CommStatus_WriteSuccess;
	return (true);
}
//==============================================================================
void MCU::genericRead(int16_t * CMD)
{
	CMD[0] = NULL;	//Register
	CMD[1] = NULL;	//Value
	CMD[2] = NULL;	//Modifier

	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_ReadNoBus;
		return;
	}

	if (bus->available() > 0){
		// Read from bus until first character; exit if not found before timeout
		if (!(bus->find(MCU_CommandStart)))
		{
			lastCommStatus = MCU_CommStatus_ReadTimeout;
			return;
		}

		// Ok we have the # now lets get the ID from the message
		readID = 0;
		int c;
		bool valid_field = false;
		while ((c = MCU::timedRead()) >= 0)
		{
			if ((c < '0') || (c > '9')) break;	// not a number character
			readID = readID * 10 + c - '0';
			valid_field = true;
		}
		if ((!valid_field))
		{
			lastCommStatus = MCU_CommStatus_ReadWrongID;
			return;
		}
		else if (readID < MCU_ID_Min || readID == BroadcastID){	// Command for LSS servo
			MCU::lssRead(readID,c);
		}
		else if (readID == mcuID){	// Command for the MCU
			//Validate the right CMD
			if (c == 'M'){
				MCU::motionRead(CMD);
			}
			else{
				lastCommStatus = MCU_CommStatus_ReadWrongIdentifier;
				return;
			}
		}
		else{
			lastCommStatus = MCU_CommStatus_ReadWrongIdentifier;
			return;
		}

		//NOT IMPLEMENTED
		// // If not a motion command check if valid
		// size_t maxLength = 5;
		// size_t index = 0;
		// while (index < maxLength)
		// {
		// 	if (c < 0 || c == MCU_CommandEnd[0])
		// 		break;
		// 	value[index] = (char) c;
		// 	index++;
		// 	c = MCU::timedRead();
		// }
		// // value[index] = '\0';
		// if (value != "RESET" || value != "CRC" || value != "QID" || value != "CID" || value != "QB" || value != "CB" || value != "Q" )
		// {
		// 	lastCommStatus = MCU_CommStatus_ReadWrongIdentifier;
		// 	return;
		// }
		// else if (value == "CID" || value == "CB"){
		// 	bool valid_value = false;
		// 	readVal = 0;
		// 	uint16_t configVal = 0;
		// 	while ((c = MCU::timedRead()) >= 0)
		// 	{
		// 		if ((c < '0') || (c > '9')) break;	// not a number character
		// 		readVal = readVal * 10 + c - '0';
		// 		valid_value = true;
		// 	}
		// 	if ((!valid_value))
		// 	{
		// 		lastCommStatus = MCU_CommStatus_ReadWrongFormat;
		// 		return;
		// 	}
		// 	else configVal = readVal;
		// 	// Return value (success)
		// 	lastCommStatus = MCU_CommStatus_ReadSuccess;
		// }
		// else if (value == "RESET" || value == "CRC" || value == "QID" || value == "QB" || value == "Q" )
		// {
		// 	//Do something
		// }
	}
	else
	{
		lastCommStatus = MCU_CommStatus_Idle;
	}
}

void MCU::lssRead(uint8_t id, int c)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_ReadNoBus;
		return;
	}

	if (bus->available() > 0){

		// Check if valid command
		size_t maxLength = 4;
		size_t index = 0;

		while (index < maxLength)
		{
			if ((c >= '0' && c <= '9') || c == MCU_CommandEnd[0]) break;
			cmd[index] = (char) c;
			index++;
			c = MCU::timedRead();
		}
		cmd[index] = '\0';
		// Check if the command is not recognized
		//	Position in Degrees			//	LED color				// LIMP				// HALT
		if (strcmp(cmd, "D") != 0 && strcmp(cmd, "LED") != 0 && strcmp(cmd, "L") != 0 && strcmp(cmd, "H") != 0)
		//if (false)
		{
			lastCommStatus = MCU_CommStatus_ReadWrongIdentifier;
			return;
		}
		else
		{
			//Get value
			index = 0;
			int16_t val;

			while (index < maxLength)
			{
				if (c < 0 || c == MCU_CommandEnd[0]) break;
				value[index] = (char) c;
				index++;
				c = MCU::timedRead();
			}
			value[index] = '\0';
			if (c < 0)
			{
				// Did not get the ending CR
				lastCommStatus = MCU_CommStatus_ReadTimeout;
			}
			else
			{
				val = valueRead(value);
				LSS::genericWrite(id,cmd,val);
			}
		}	
	}
	else
	{
		lastCommStatus = MCU_CommStatus_Idle;
	}
}

void MCU::motionRead(int16_t * motionCMD)
{
	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_ReadNoBus;
		return;
	}

	if (bus->available() > 0){
		int c;
		bool valid_field = 0;

		while ((c = MCU::timedRead()) >= 0)
		{
			if ((c < '0') || (c > '9')) break;	// not a number character
			motionCMD[0] = motionCMD[0] * 10 + c - '0';
			valid_field = true;
		}

		if ((!valid_field) || (motionCMD[0] >= 20))
		{
			lastCommStatus = MCU_CommStatus_ReadUnknown;
		}
	
		// If it is a special move
		else if (c == MCU_CommandEnd[0])
		{
			if (motionCMD[0] == 0) motionCMD[1] = NULL; //Stop
			// Return value (success)
			lastCommStatus = MCU_CommStatus_ReadSuccess;
		}
		//if it isn't a special move
		else if (c == 'V')
		{
			//Get value
			size_t maxLength = 4;
			size_t index = 0;

			while (index < maxLength)
			{
				c = MCU::timedRead();
				if (c < 0 || c == MCU_CommandEnd[0] || c == 'S')
					break;
				value[index] = (char) c;
				index++;
			}
			value[index] = '\0';

			if (c < 0) 
			{
				// did not get the ending CR
				lastCommStatus = MCU_CommStatus_ReadTimeout;
				motionCMD[1] = NULL;
			}
			else motionCMD[1] = valueRead(value);

			bool valid_speed = false;
			readVal = 0;
			if (c == 'S')
			{
				while ((c = MCU::timedRead()) >= 0)
				{
					if ((c < '0') || (c > '9')) break;	// not a number character
					readVal = readVal * 10 + c - '0';
					valid_speed = true;
				}
			}
			if ((!valid_speed))
			{
				lastCommStatus = MCU_CommStatus_ReadUnknown;
				motionCMD[2] = NULL;
			}
			else motionCMD[2] = readVal;
			// Return value (success)
			lastCommStatus = MCU_CommStatus_ReadSuccess;
		}
		else
		{
			lastCommStatus = MCU_CommStatus_ReadWrongIdentifier;
			return;
		}
	}
	else
	{
		lastCommStatus = MCU_CommStatus_Idle;
	}
}

//==============================================================================
int16_t MCU::valueRead(const char * valueStr)
{
	// Let the string function do all of the main parsing work.
	if (valueStr == (char *) nullptr)
	{
		// the above method will have already set the error condition. 
		return 0;
	}

	// Exit condition
	if (bus == (Stream*) nullptr)
	{
		lastCommStatus = MCU_CommStatus_ReadNoBus;
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
		lastCommStatus = MCU_CommStatus_ReadWrongID;
		return (0);
	}
	// return the computed value
	return value * value_sign; 
} 


// -- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Public functions (instance) ---- ---- ---- ---- ---- ---- ---- ---- ---- ----

// Get current servo ID
uint8_t MCU::getID(void)
{
	return (mcuID);
}

// Set current servo ID (guards for safe range of [MCU_ID_Min, MCU_ID_Max])
void MCU::setID(uint8_t id)
{
	if (id < MCU_ID_Min)
		id = MCU_ID_Min;
	if (id > MCU_ID_Max)
		id = MCU_ID_Max;
	mcuID = id;
}

MCU_LastCommStatus MCU::getLastCommStatus(void)
{
	return (MCU::lastCommStatus);
}

// --- Actions ---

// Reset the MCU
// Note: no waiting is done here. MCU will take a bit more than a second to reset/start responding to commands.
bool MCU::reset(void)
{
	return (MCU::genericWrite(this->mcuID, MCU_ActionReset));
}

// //> Queries
// /*
//  // Not implemented since they serve no purpose; the IDs and baud are fixed by design
//  uint8_t getID(void){}
//  uint8_t getBaud(void){}
//  */
// // Returns current status
// MCU_Status MCU::getStatus(void)
// {
// 	// Variables
// 	MCU_Status value = MCU_StatusUnknown;

// 	// Ask servo for status; exit if it failed
// 	if (!(MCU::genericWrite(this->mcuID, MCU_QueryStatus)))
// 	{
// 		MCU::lastCommStatus = MCU_CommStatus_WriteNoBus;
// 		return (value);
// 	}

// 	// Read response from servo
// 	value = (MCU_Status) valueRead(MCU::genericRead(this->mcuID, MCU_QueryStatus));

// 	// Return result
// 	return (value);
// }

// // Returns speed in (1/10°)/s
// uint8_t MCU::getSpeed(void)
// {
// 	// Variables
// 	uint8_t value = 0;

// 	// Ask servo for status; exit if it failed
// 	if (!(MCU::genericWrite(this->mcuID, MCU_QuerySpeed)))
// 	{
// 		MCU::lastCommStatus = MCU_CommStatus_WriteNoBus;
// 		return (value);
// 	}

// 	// Read response from servo
// 	value = (uint8_t) valueRead(MCU::genericRead(this->mcuID, MCU_QuerySpeed));

// 	// Return result
// 	return (value);
// }

// MCU_Model MCU::getModel(void)
// {
// 	// Variables
// 	char * valueStr;

// 	// Ask servo for status; exit if it failed
// 	if (!(MCU::genericWrite(this->mcuID, MCU_QueryModelString)))
// 	{
// 		MCU::lastCommStatus = MCU_CommStatus_WriteNoBus;
// 		return (MCU_ModelUnknown);
// 	}

// 	// Read response from servo
// 	valueStr = MCU::genericRead(this->mcuID, MCU_QueryModelString);

// 	if (strcmp(valueStr, MCU_MODEL_2IO) == 0)
// 	{
// 		return (MCU_Model2IO);
// 	}
// 	else if (strcmp(valueStr, MCU_MODEL_BOT) == 0)
// 	{
// 		return (MCU_ModelBOT);
// 	}
// 	else if (strcmp(valueStr, MCU_MODEL_RPI) == 0)
// 	{
// 		return (MCU_ModelRP2040);
// 	}
// 	else if (strcmp(valueStr, MCU_MODEL_ESP) == 0)
// 	{
// 		return (MCU_ModelESP32);
// 	}
// 	else
// 	{
// 		return (MCU_ModelUnknown);
// 	}
// }

// uint16_t MCU::getDistance(MCU_QueryTypeDistance queryTypeDistance)
// {
// 	// Variables
// 	uint16_t value = 0;

// 	// Ask servo for status; exit if it failed
// 	if (!(MCU::genericWrite(this->mcuID, MCU_QueryAnalog, queryTypeDistance)))
// 	{
// 		MCU::lastCommStatus = MCU_CommStatus_WriteNoBus;
// 		return (value);
// 	}

// 	// Read response from servo
// 	value = (uint16_t) valueRead(MCU::genericRead(this->mcuID, MCU_QueryAnalog));

// 	// Return result
// 	return (value);
// }

// bool MCU::setMode(MCU_ConfigMode value)
// {
// 	return (MCU::genericWrite(this->mcuID, MCU_ConfigModeRC, value));
// }
