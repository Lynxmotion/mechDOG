/*
    Author:     Eduardo Nunes
                Geraldine Barreto
    Version:    1.0
    Licence:    LGPL-3.0 (GNU Lesser General Public License)
    Description:  Basic example of control of LSS based quadruped robot.
*/

#include "src/Quadruped.h"

#define MOVE_TIME 4000 //ms Time between movements for API example
#define WIFI_TIME 25000
#define LSS_BAUD 38400
#define MCU_BAUD 38400
#define LSS_SERIAL  (Serial)

#if QUADRUPED_CONTROL == C_WIFI
  //                        Rx,Tx
  SoftwareSerial BEE_SERIAL(10,11);  //3,2(Arduino UNO, BotBoarduino) //10,11(2IO)

  String ssid_command = "AT+SSID=";
  String pass_command = "AT+PASSWORD=";
  const char ssid[] = {ssid_command.concat(WSSID)};        // WiFi SSID
  const char passwd[] = {pass_command.concat(WPASS)};      // WiFi  password
  
  void xbeeConfig(void){
    Serial.println("Sending Config commands");
    delay(100);
    BEE_SERIAL.print("+++");
    delay(1000);
    BEE_SERIAL.println("AT+BAUDRATE=" + MCU_BAUD);
    delay(1000);
    BEE_SERIAL.println(ssid);
    delay(100);
    BEE_SERIAL.println(passwd);
    delay(100);
    BEE_SERIAL.println("AT+REBOOT");
    Serial.println("Xbee Restarted");
  }

  DTime dt = DTime(WIFI_TIME);
  char rd;
  
  void showSerial(void){
    LSS_SERIAL.println("Waiting 25 sec for IP");
    while(!dt.getDT()){
    if(BEE_SERIAL.available()>0){
      rd = BEE_SERIAL.read();
      LSS_SERIAL.print(rd);
      }
    }
    LSS_SERIAL.println("Done");
  }
#endif

bool sample_sequence = false;
Quadruped robot = Quadruped(MechDog);
uint8_t state = 0;
DTime state_time = DTime(MOVE_TIME);
void sampleMoveSequence(){
  switch (state)
  {
    case 0:
      robot.height(140); //mm
      break;
    case 1:
      state_time.updateDT(1000);
      robot.pitch(20);
      robot.roll(20);
      break;
    case 2:
      robot.roll(-20);
      break;
    case 3:
      robot.roll(0);
      robot.pitch(0);
      break;
    case 4:
      state_time.updateDT(1500);
      robot.yaw(15);
      robot.height(120);
      break;
    case 5:
      robot.yaw(-15);
      robot.height(140);
      break;
    case 6:
      state_time.updateDT(5000);
      robot.specialMove(SIT);
      break;
    case 7:
      state_time.updateDT(5000);
      robot.specialMove(WIGGLE);
      break;
    case 8:
      state_time.updateDT(1000);
      robot.specialMove(UP);
      break;
    case 9:
      state_time.updateDT(4000);
      robot.setSpeed(3);
      robot.rotate(CCW);
      break;
    case 10:
      state_time.updateDT(1500);
      robot.setSpeed(3);
      robot.rotate(StopRotation);
      break;
    case 11:
      state_time.updateDT(3000);
      robot.setSpeed(4);
      robot.rotate(CW);
      break;
    case 12:
      state_time.updateDT(2000);
      robot.setSpeed(3);
      robot.rotate(StopRotation);
      break;
    default:
      break;
  }
  state ++;
  if(state > 12) state = 0;
}


void setup()
{
  //Verify that the Baudrate used is the same as the one configured in the LSS servos.
  robot.initServoBus(LSS_SERIAL, LSS_BAUD);         //LSS bus w/ Hardware serial
  delay(200);
  LSS(254).setColorLED(2);                          //Green (Check communication)

  #if QUADRUPED_CONTROL == C_RC
    //RC mode w/ analog pin, default pin A0, invert false
    robot.initMCUBus(RC, A3);                       
  #elif QUADRUPED_CONTROL == C_WIFI
    //WiFi RC mode w/ Software serial
    robot.initMCUBus(WifiRC, BEE_SERIAL, MCU_BAUD);     
    /*Uncomment the following instructions to view on the serial terminal 
    the IP assigned to the Xbee*/    
//  delay(100);
//  xbeeConfig();
//  showSerial();
    /*
    For more information about xbee configuration go to the WIKI
    */
  #else
    sample_sequence = true;
  #endif
}

void loop()
{
  if(state_time.getDT() && sample_sequence){
    sampleMoveSequence();
  }
  robot.loop();
}
