/*
    Author:     Eduardo Nunes
                Geraldine Barreto
    Version:    1.0
    Licence:    LGPL-3.0 (GNU Lesser General Public License)
    Description:  Basic example of control of  LSS based quadruped robot.
*/

#include "src/Quadruped.h"

#define WIFI_TIME 25000
#define LSS_BAUD 38400
#define LSS_SERIAL  (Serial)
char rd;

#ifdef MCU_SupportSoftwareSerial
//                       Rx,Tx
SoftwareSerial BEE_SERIAL(10,11);  //3,2 (Arduino UNO, BotBoarduino) //10,11 (2IO)

const char ssid[] = {"AT+SSID=Robotech"};              //{"AT+SSID=Barreto"};              // WiFi SSID
const char passwd[] = {"AT+PASSWORD=deg.1812"};        //{"AT+PASSWORD=15091994201723"};   // WiFi  password

void xbeeConfig(void){
    Serial.println("Sending Config commands");
    delay(100);
    BEE_SERIAL.print("+++");
    delay(1000);
    BEE_SERIAL.println("AT+BAUDRATE=" + LSS_BAUD);
    delay(1000);
    BEE_SERIAL.println(ssid);
    delay(100);
    BEE_SERIAL.println(passwd);
    delay(100);
    BEE_SERIAL.println("AT+REBOOT");
    Serial.println("Xbee Restarted");
 }

DTime dt = DTime(WIFI_TIME);

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

Quadruped robot = Quadruped(MechDog);

void setup()
{
   delay(1500);                                 //Let the LSS's booting up
   robot.initServoBus(LSS_SERIAL, LSS_BAUD);    //LSS bus w/ Hardware serial
   delay(200);
   LSS(254).setColorLED(2);                     //Green

   //RC mode w/ analog pin, default pin A0, invert false
   robot.initMCUBus(RC, A3);                       

   //WiFi RC mode w/ Software serial
//   robot.initMCUBus(WifiRC, BEE_SERIAL, LSS_BAUD);         
//   delay(100);
//   xbeeConfig();
//   showSerial();
}

void loop()
{
  robot.loop();
}
