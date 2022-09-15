/*
 *	Authors:		Eduardo Nunes
 *					Geraldine Barreto
 *	Version:		1.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License)
 *	
 *	Description:	A library that controls a quadruped robot that uses the LSS servos.
 *					Offers support for both Arduino Uno, Mega and others through
 *					the use of the Stream class for communication.
 */

#ifndef QUADRUPED_H
#define QUADRUPED_H

#include "LSS.h"
#include "LSS_MCU.h"
#include "IK_quad.h"
#include "Utils.h"

#ifdef MCU_SupportPPM
#include "ppm.h"
#endif

#define SpecialMoveSpeed 0
#define StopMoveSpeed 5

enum ControlMode{
    NoControlSelected,
    //WifiStreaming,
    WifiRC,
    RC,
};

enum RCSwitchMode{
    OffsetMode,
    WalkingMode,
    RPYMode,
};

enum RCchannels{
    ROLL = 1,
    PITCH,
    SPEED,
    YAW,
    SW1,
    BUTTON,
    SW2,
    HEIGHT,
};

class Quadruped
{
    public:

    int16_t last_cmd[3];
    bool move_flag = true;
    ControlMode ctrlSelected = NoControlSelected;
    RCSwitchMode RC_mode;

    Quadruped(LSS_Robot_Model robot = MechDog);
    ~Quadruped(void);
    
    void initServoBus(HardwareSerial &s, uint32_t baud);
    void initMCUBus(ControlMode ctrl, HardwareSerial &s, uint32_t baud);
#ifdef MCU_SupportSoftwareSerial
    void initMCUBus(ControlMode ctrl, SoftwareSerial &s, uint32_t baud);
#endif
#ifdef MCU_SupportPPM
    void initMCUBus(ControlMode ctrl, int pin = A0, bool invert = false);
#endif
    void walk(int16_t angle);
    void rotate(Rotation_Dir dir);
    void frontalOffset(int16_t mm); // frontal
    void lateralOffset(int16_t mm);  // lateral
    void height(int16_t mm);
    void pitch(int16_t angle);
    void roll(int16_t angle);
    void yaw(int16_t angle);
    void gaitType(Foot_Trajectory type = Circular);
    int8_t getSpeed(void);
    void specialMove(Special_Moves move = UP);
    void loop(void); 
    LSS_Robot_Model getRobotModel(void);
    void setSpeed(uint8_t speed);
    void readControl(void);
    
    private:
    Body robot; 
    DTime dt = DTime(100);
    int8_t speed = 1, actual_speed;
    void triggerMotion(bool debug);
    void changeSpeed(int8_t speed);
    void readSerial(void);
#ifdef MCU_SupportPPM
    void readPPM(void);
#endif

};


#endif
