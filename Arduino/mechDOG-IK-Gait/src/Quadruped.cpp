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

#include "Quadruped.h"

Quadruped::Quadruped(LSS_Robot_Model robot_model = MechDog){
    this->robot = Body(robot_model);
    this->changeSpeed(this->speed);
}

Quadruped::~Quadruped(void){}

void Quadruped::initServoBus(HardwareSerial &s, uint32_t baud){
    LSS::initBus(s, baud);
     //Settings
    LSS(11).setGyre(1);
    LSS(12).setGyre(-1);
    LSS(13).setGyre(-1);

    LSS(21).setGyre(1);
    LSS(22).setGyre(-1);
    LSS(23).setGyre(-1);

    LSS(31).setGyre(-1);
    LSS(32).setGyre(1);
    LSS(33).setGyre(1);

    LSS(41).setGyre(-1);
    LSS(42).setGyre(1);
    LSS(43).setGyre(1);

    LSS(254).setMotionControlEnabled(false);
    LSS(254).setAngularHoldingStiffness(1);
    LSS(254).setAngularStiffness(-2);
}

void Quadruped::initMCUBus(ControlMode ctrl, HardwareSerial &s, uint32_t baud){
    if (ctrl == RC) this->ctrlSelected = NoControlSelected;
    else{
        this->ctrlSelected = ctrl; 
        MCU::initBus(s, baud);
    }
}

#ifdef MCU_SupportSoftwareSerial
void Quadruped::initMCUBus(ControlMode ctrl, SoftwareSerial &s, uint32_t baud){
    if (ctrl == RC) this->ctrlSelected = NoControlSelected;
    else{
        this->ctrlSelected = ctrl; 
        MCU::initBus(s, baud);
    }
}
#endif

#ifdef MCU_SupportPPM
void Quadruped::initMCUBus(ControlMode ctrl, int pin, bool invert){
    uint8_t sw1_mode;
    if (ctrl == RC){
        this->ctrlSelected = ctrl; 
        ppm.begin(pin, invert);
        sw1_mode = round((ppm.read_channel(SW1)-1000)/500.0);
        LSS(254).setColorLED(sw1_mode+3);
            
    }
    else this->ctrlSelected = NoControlSelected;
}
#endif

void Quadruped::walk(int16_t angle){
    this->move_flag = true;
    this->robot.update_flag = true;
    if(angle == 0){
        this->robot.new_move_state = 0;
        if(this->robot.new_jog_mode) this->robot.new_move_state = 1;
    }else{
        this->robot.new_move_state = 1;
    }
    this->robot.new_director_angle = angle;

    if(this->robot.stopped && this->robot.new_move_state == StopWalk){
        this->changeSpeed(StopMoveSpeed);
    }else if(this->speed != this->actual_speed){        
        this->changeSpeed(this->speed);
    }   
}

void Quadruped::rotate(Rotation_Dir dir){
    this->move_flag = true;
    this->robot.update_flag = true;
    this->robot.new_rot_angle = dir;
    if(this->speed != this->actual_speed){        
        this->changeSpeed(this->speed);
    }
}

void Quadruped::frontalOffset(int16_t mm){
    this->move_flag = true;
    if(this->robot.stopped && this->robot.sp_move == UP){
        if(mm >= Body::cgx_limits[min] && mm <=Body::cgx_limits[max]) this->robot.cgx = mm;
        if(StopMoveSpeed!= this->actual_speed) this->changeSpeed(StopMoveSpeed);
    }
}

void Quadruped::lateralOffset(int16_t mm){
    this->move_flag = true;
    if(this->robot.stopped && this->robot.sp_move == UP){
        if(mm >= Body::cgz_limits[min] && mm <=Body::cgz_limits[max]) this->robot.cgz = mm;
        if(StopMoveSpeed!= this->actual_speed) this->changeSpeed(StopMoveSpeed);
    }
}

void Quadruped::height(int16_t mm){
    this->move_flag = true;
    if(this->robot.sp_move == UP && mm >= Body::cgy_limits[min] && mm <=Body::cgy_limits[max]) this->robot.cgy = mm;
    if(this->robot.stopped){
        if(StopMoveSpeed!= this->actual_speed) this->changeSpeed(StopMoveSpeed);
    }
}

void Quadruped::pitch(int16_t angle){
    this->move_flag = true;
    int16_t angle_rads = RADS(angle);
    if(this->robot.sp_move == UP && angle >= Body::pitch_limits[min] && angle <= Body::pitch_limits[max]) this->robot.pitch = RADS(angle);
    if(this->robot.stopped){
        if(StopMoveSpeed!= this->actual_speed) this->changeSpeed(StopMoveSpeed);
    }
}

void Quadruped::roll(int16_t angle){
    this->move_flag = true;
    int16_t angle_rads = RADS(angle);
    if(this->robot.sp_move == UP && angle >= Body::roll_limits[min] && angle <= Body::roll_limits[max]) this->robot.roll = RADS(angle);
    if(this->robot.stopped){
        if(StopMoveSpeed!= this->actual_speed) this->changeSpeed(StopMoveSpeed);
    }
}

void Quadruped::yaw(int16_t angle){
    this->move_flag = true;
    if(this->robot.sp_move == UP && angle >= Body::yaw_limits[min] && angle <= Body::yaw_limits[max]) this->robot.yaw = RADS(angle);
    if(this->robot.stopped){
        if(StopMoveSpeed != this->actual_speed) this->changeSpeed(StopMoveSpeed);
    }
}

void Quadruped::specialMove(Special_Moves move = UP){
    if(move == JOG_ON){
        if(!this->robot.new_jog_mode){
            this->robot.new_sp_move = UP;
            this->robot.new_jog_mode = true; 
            this->walk(0);
        }
    }
    else if(move == JOG_OFF){
        if(this->robot.new_jog_mode){
            this->robot.new_sp_move = UP;
            this->robot.new_jog_mode = false; 
            this->walk(0);
        }
    }
    else if(this->robot.new_sp_move != move){
        this->robot.new_sp_move = move;
        this->move_flag = true;
        this->robot.update_flag = true;
        this->robot.new_move_state = StopWalk;
        this->robot.new_rot_angle = StopRotation;
    }
}

void Quadruped::gaitType(Foot_Trajectory type = Circular){
    this->move_flag = true;
    this->robot.trajectory_type = type;
}

int8_t Quadruped::getSpeed(void){
    return this->speed;
}

void Quadruped::setSpeed(uint8_t speed){
    this->speed = speed;
}

void Quadruped::changeSpeed(int8_t speed){
    this->move_flag = true;
    this->actual_speed = speed;
    switch (speed)
    {
        case StopMoveSpeed:
            this->dt.updateDT(60);
            if(this->speed == 4){this->robot.new_beta = Dynamic;}
            else{this->robot.new_beta = Static;}
            LSS::LSS(254).setFilterPositionCount(14);
            break;
        case SpecialMoveSpeed:
            this->dt.updateDT(180);
            LSS::LSS(254).setFilterPositionCount(14);
            break;
        case 1:
            this->dt.updateDT(70);
            this->robot.new_beta = Static;
            LSS::LSS(254).setFilterPositionCount(4); 
            break;
        case 2:
            this->dt.updateDT(60);
            this->robot.new_beta = Static;
            LSS::LSS(254).setFilterPositionCount(4);
            break;
        case 3:
            this->dt.updateDT(50);
            this->robot.new_beta = Static;
            LSS::LSS(254).setFilterPositionCount(3);
            break;
        case 4:
            this->dt.updateDT(55);
            this->robot.new_beta = Dynamic;
            LSS::LSS(254).setFilterPositionCount(3);
            break;
        default:
            break;
    }
}

LSS_Robot_Model Quadruped::getRobotModel(void){
    return this->robot.model;
}

void Quadruped::triggerMotion(bool debug){
    
    if(this->last_cmd[2]!= NULL){
        this->setSpeed(this->last_cmd[2]);
    } 

    switch(this->last_cmd[0]){
        case MCU_Walking:
            this->walk(this->last_cmd[1]);
            break;
        case MCU_Rotation:
            this->rotate(this->last_cmd[1]);
            break;
        case MCU_Roll:
            this->roll(this->last_cmd[1]);
            break;
        case MCU_Pitch:
            this->pitch(this->last_cmd[1]);
            break;
        case MCU_Yaw:
            this->yaw(this->last_cmd[1]);
            break;
        case MCU_FrontalOffset:
            this->frontalOffset(this->last_cmd[1]);
            break;
        case MCU_Height:
            this->height(this->last_cmd[1]);
            break;
        case MCU_LateralOffset:
            this->lateralOffset(this->last_cmd[1]);
            break;
        case MCU_GaitType:
            this->gaitType(this->last_cmd[1]);
            break;
        case MCU_Up:
            this->specialMove(UP);
            break;
        case MCU_Sit:
            this->specialMove(SIT);
            break;
        case MCU_Lay:
            this->specialMove(LAY);
            break;
        case MCU_Paw:
            this->specialMove(PAW);
            break;
        case MCU_Wiggle:
            this->specialMove(WIGGLE);
            break;
        case MCU_Tinkle:
            this->specialMove(TINKLE);
            break;
        case MCU_Stretch:
            this->specialMove(STRETCH);
            break;
        case MCU_Jog_On:
            this->specialMove(JOG_ON);
            break;
        case MCU_Jog_Off:
            this->specialMove(JOG_OFF);
            break;
        default:
            this->walk(StopWalk);
            this->rotate(StopRotation);
    }
    if(debug){
        Serial.print("CMD: ");
        Serial.println(last_cmd[0]);
        Serial.print("V: ");
        Serial.println(last_cmd[1]);
        Serial.print("S: ");
        Serial.println(last_cmd[2]);
    }
}

void Quadruped::readSerial(void){
    //MCU::MCU().motionRead(this->last_cmd);
    MCU::MCU().genericRead(this->last_cmd);
    MCU_LastCommStatus status = MCU().getLastCommStatus();
    if (status != 0) Serial.println(status);
    switch (status)
    {
        case MCU_CommStatus_Idle:
            /* code */
            break;
        case MCU_CommStatus_ReadSuccess:
            this->triggerMotion(false);
            break;
        case MCU_CommStatus_ReadTimeout:
            /* code */
            break;
        case MCU_CommStatus_ReadNoBus:
            /* code */
            break;
        default:
            break;
    }
}

#ifdef MCU_SupportPPM
void Quadruped::readPPM(void){
    Special_Moves move;
    int16_t angY, angX, y;
    int angle = this->robot.new_director_angle;
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t yaw = 0; 
    int x = this->robot.cgx;
    int z = this->robot.cgz;
    int8_t rot = this->robot.new_rot_angle;
    int16_t cgx_offset = 0;
    bool button = false;
    uint8_t sw1_mode = round((ppm.read_channel(SW1)-1000)/500.0);
    uint8_t sw2_mode = round((ppm.read_channel(SW2)-1000)/500.0);
    uint8_t speedRC = round((ppm.read_channel(SPEED)-500)/500.0);
    if (ppm.read_channel(BUTTON) > 1500) button = true; ///speedRC = 4;                   //Dynamic gait
    if (sw1_mode != this->RC_mode) LSS(254).setColorLED(sw1_mode+3);

    switch(sw1_mode){
        case OffsetMode:
            z = -1*constrain(map(ppm.read_channel(ROLL), 1000, 2000, Body::cgz_limits[min], Body::cgz_limits[max]), Body::cgz_limits[min], Body::cgz_limits[max]);
            x = -1*constrain(map(ppm.read_channel(PITCH), 1000, 2000, Body::cgx_limits[min], Body::cgx_limits[max]), Body::cgx_limits[min], Body::cgx_limits[max]);
            y = constrain(map(ppm.read_channel(HEIGHT), 1000, 2000, Body::cgy_limits[min], Body::cgy_limits[max]), Body::cgy_limits[min], Body::cgy_limits[max]);
            yaw = constrain(map(ppm.read_channel(YAW), 1000, 2000, Body::yaw_limits[min], Body::yaw_limits[max]), Body::yaw_limits[min], Body::yaw_limits[max]);
            switch(sw2_mode){
                case 0:
                    move = TINKLE;
                    break;
                case 2:
                    move = LAY;
                    break;
                default:
                    move = UP;
                    if(this->robot.sp_move == UP){
                        if (abs(this->robot.cgx - (x + CGX_MECHDOG_STATIC)) > 1) this->frontalOffset(x+CGX_MECHDOG_STATIC);
                        if (abs(this->robot.cgz - z) > 1) this->lateralOffset(z);
                        if (abs(this->robot.cgy - y) > 2) this->height(y);
                        if (abs(DEG(this->robot.yaw)- yaw) > 1) this->yaw(yaw);
                    }
                    if(button && !this->robot.new_jog_mode) {
                        move = JOG_ON;
                        this->setSpeed(3);
                    }else if (!button && this->robot.new_jog_mode){
                        move = JOG_OFF;
                    }
                   break;
                }
            break;
        case WalkingMode:
            //Mix roll and pitch to get walking angle
            angY  = ppm.read_channel(ROLL) - 1500;
            angX = 1500 - ppm.read_channel(PITCH);
            if (abs(angY) > 250 || abs(angX) > 250) {
                angle = atan2(angY,angX)*4068/71;
                if (angle <= 0) angle = 360 + angle;
            }
            else angle = 0;

            switch(sw2_mode){
                case 0:
                    move = SIT;
                    break;
                case 2: 
                    move = STRETCH;
                    break;
                default:
                    move = UP;
                    if(button) speedRC = 4;
                    if(this->robot.sp_move == UP ){
                        if(speedRC != this->speed) {                                    //Change Servo color to red in Dynamic walk
                            if(speedRC == 4) LSS(254).setColorLED(1);
                            else if(this->speed == 4) LSS(254).setColorLED(sw1_mode+3); 
                            this->setSpeed(speedRC);  
                            this->walk(angle);
                        }
                        if (abs(this->robot.new_director_angle - angle) >= 10) {
                            this->walk(angle);
                        }
                        rot = round((ppm.read_channel(YAW)-1500)/500.0);
                        if (this->robot.new_rot_angle != rot) this->rotate(rot);
                        y = constrain(map(ppm.read_channel(HEIGHT), 1000, 2000, Body::cgy_limits[min], Body::cgy_limits[max]), Body::cgy_limits[min], Body::cgy_limits[max]);
                        if (abs(this->robot.cgy - y) > 2 && this->robot.sp_move == UP)  this->height(y);
                    }
                    break;
            }
            break;
        case RPYMode:
            roll = constrain(map(ppm.read_channel(ROLL), 1000, 2000, Body::roll_limits[min], Body::roll_limits[max]), Body::roll_limits[min], Body::roll_limits[max]);
            pitch = -constrain(map(ppm.read_channel(PITCH), 1000, 2000, Body::pitch_limits[min], Body::pitch_limits[max]),Body::pitch_limits[min], Body::pitch_limits[max]);
            yaw = constrain(map(ppm.read_channel(YAW), 1000, 2000, Body::yaw_limits[min], Body::yaw_limits[max]), Body::yaw_limits[min], Body::yaw_limits[max]);

            switch(sw2_mode){
                case 0:
                    move = PAW;
                    break;
                case 2: 
                    move = WIGGLE;
                    break;
                default:
                    move = UP;
                    if(this->robot.sp_move == UP ){
                        if (abs(DEG(this->robot.roll) - roll)> 1) this->roll(roll);
                        if (abs(DEG(this->robot.pitch) - pitch) > 1) this->pitch(pitch);
                        if (abs(DEG(this->robot.yaw)- yaw) > 1) this->yaw(yaw);

                        y = constrain(map(ppm.read_channel(HEIGHT), 1000, 2000, Body::cgy_limits[min], Body::cgy_limits[max]), Body::cgy_limits[min], Body::cgy_limits[max]);
                        if (abs(this->robot.cgy - y) > 2 && this->robot.sp_move == UP)  this->height(y);
                    }
                    break;
            }
            break;
    }
    this->RC_mode = sw1_mode;
    this->specialMove(move);
}
#endif

void Quadruped::loop(void){
    if(this->dt.getDT()){
        this->readControl();
        if(this->move_flag || !this->robot.stopped){
            if(this->robot.sp_move == UP) {
                this->robot.walk(); // if up and balance option with IMU
                this->robot.joints.moveServos();
                if(this->robot.sp_move != UP) this->changeSpeed(SpecialMoveSpeed);
            }
            if(this->robot.sp_move != UP){
                this->robot.specialMoves();
                this->robot.joints.moveServos();
                if(this->robot.sp_move == UP) this->changeSpeed(StopMoveSpeed);
            } 
            this->move_flag = false;
        }
        //Serial.print(">>>>>>>>  ");
        //this->dt.getDT(true);
    }	
}

void Quadruped::readControl(void){

    switch (this->ctrlSelected)
    {
    case NoControlSelected:
        /* code */
        break;
    // case WifiStreaming:
    //     this->readStreaming();
    //     break;
    case WifiRC:
        this->readSerial();
        break;
    case RC:
        #ifdef MCU_SupportPPM
        this->readPPM();
        #endif
        break;
    default:
        break;
    }
}