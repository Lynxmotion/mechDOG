/*
  Author: Edu Dickel (https://www.youtube.com/mechdickel)
  Description: code of walking gait (trot), turnings, side trotting and demo poses for mechDOG (quadruped robot/robot dog).
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LSS.h>

struct DATA_STRUCTURE
{
  int joystickB; // button
  int joystickX;
  int joystickY;
  int joystickZ;
  int button1;
  int button2;
  int button3;
  int button4;
  int toggle1;
  int toggle2;
  int toggle3;
};

DATA_STRUCTURE data;

RF24 radio(9, 10); // CE, CSN
const uint64_t dataPipe = 0x646F67676FLL;

// RR = right rear; RF = right front; LR = left rear; LF = left front
LSS myLSS11 = LSS(11); // RR hip roll
LSS myLSS12 = LSS(12); // RR hip pitch
LSS myLSS13 = LSS(13); // RR knee
LSS myLSS21 = LSS(21); // RF hip roll
LSS myLSS22 = LSS(22); // RF hip pitch
LSS myLSS23 = LSS(23); // RF knee
LSS myLSS31 = LSS(31); // LR hip roll
LSS myLSS32 = LSS(32); // LR hip pitch
LSS myLSS33 = LSS(33); // LR knee
LSS myLSS41 = LSS(41); // LF hip roll
LSS myLSS42 = LSS(42); // LF hip pitch
LSS myLSS43 = LSS(43); // LF knee

float initPos11 =      -125;
float initPos12 =      -295;
float initPos13 =       180;
float initPos21 = initPos11;
float initPos22 = initPos12;
float initPos23 = initPos13;
float initPos31 = initPos11;
float initPos32 = initPos12;
float initPos33 = initPos13;
float initPos41 = initPos11;
float initPos42 = initPos12;
float initPos43 = initPos13;

float currPos11, prevPos11, targPos11;
float currPos12, prevPos12, targPos12;
float currPos13, prevPos13, targPos13;
float currPos21, prevPos21, targPos21;
float currPos22, prevPos22, targPos22;
float currPos23, prevPos23, targPos23;
float currPos31, prevPos31, targPos31;
float currPos32, prevPos32, targPos32;
float currPos33, prevPos33, targPos33;
float currPos41, prevPos41, targPos41;
float currPos42, prevPos42, targPos42;
float currPos43, prevPos43, targPos43;

float offset11 =    0;
float offset12 = -450;
float offset13 =  900;
float offset21 =    0;
float offset22 = -450;
float offset23 =  900;
float offset31 =    0;
float offset32 = -450;
float offset33 =  900;
float offset41 =    0;
float offset42 = -450;
float offset43 =  900;

byte i;   // "increment()" counter
byte ird; // increment rate divider
byte state;
byte prevState;
byte phase = 1;
byte phaseDelay = 50;
byte queryDelay = 2;

bool wakeUpDone = false; // to indicate if "wakeUp()" has reached its target positions ("targPos")
bool moving = false;     // indicates if it is running a motion function

void setup()
{
  radio.begin();
  radio.openReadingPipe(1, dataPipe);
  radio.startListening();

  LSS::initBus(Serial, 115200);

  // Gyre Direction
  Serial.write("#11CG1\r#12CG1\r#13CG1\r");    // RR leg servos = CW
  Serial.write("#21CG1\r#22CG1\r#23CG1\r");    // RF leg servos = CW
  Serial.write("#31CG-1\r#32CG-1\r#33CG-1\r"); // LR leg servos = CCW
  Serial.write("#41CG-1\r#42CG-1\r#43CG-1\r"); // LF leg servos = CCW

  // Enable Motion Profile
  Serial.write("#254CEM0\r");  // EM0 = motion profile disabled

  // Filter Position Count
  Serial.write("#254FPC5");    // default = 5

  // Angular Stiffness
  Serial.write("#254CAS-4\r"); // default = 0

  // Angular Holding Stiffness
  Serial.write("#254CAH2\r");  // default = 4

  // LED Color
  Serial.write("#254CLED3\r"); // 3 = blue
  // Serial.write("#254CLB0\r");

  Serial.write("#254RESET\r");
  delay(2000); // wait 2 sec for reboot

  queryPos();  // get positions at startup
  setPrevPos();
}

void loop()
{
  if (wakeUpDone == false)
  {
    wakeUp();
  }

  else if (wakeUpDone == true)
  {
    stateMachine();
    switch (state)
    {
      case 0:
        Stop();   // move to initial positions ("initPos")
        break;

      case 1:
        trot_F(); // trot forward
        break;

      case 2:
        trot_B(); // trot backward
        break;

      case 3:
        trot_R(); // side trot to the right
        break;

      case 4:
        trot_L(); // side trot to the left
        break;

      case 5:
        trot_P(); // trot in place
        break;

      case 6:
        turn_R(); // turn right
        break;

      case 7:
        turn_L(); // turn left
        break;

      case 8:
        look_U(); // look up
        break;

      case 9:
        look_D(); // look down
        break;

      case 10:
        roll_R(); // roll to the right (not real kinematics, just demo pose)
        break;

      case 11:
        roll_L(); // roll to the left (not real kinematics, just demo pose)
        break;

      case 12:
        rest();   // lie down
        break;
    }
  }
}

void queryPos() // query positions and set them as current positions ("currPos")
{
  currPos11 = myLSS11.getPosition(); delay(queryDelay);
  currPos12 = myLSS12.getPosition(); delay(queryDelay);
  currPos13 = myLSS13.getPosition(); delay(queryDelay);
  currPos21 = myLSS21.getPosition(); delay(queryDelay);
  currPos22 = myLSS22.getPosition(); delay(queryDelay);
  currPos23 = myLSS23.getPosition(); delay(queryDelay);
  currPos31 = myLSS31.getPosition(); delay(queryDelay);
  currPos32 = myLSS32.getPosition(); delay(queryDelay);
  currPos33 = myLSS33.getPosition(); delay(queryDelay);
  currPos41 = myLSS41.getPosition(); delay(queryDelay);
  currPos42 = myLSS42.getPosition(); delay(queryDelay);
  currPos43 = myLSS43.getPosition(); delay(queryDelay);
}

void wakeUp()
{
  moving = true;
  ird = 100;

  if (phase == 1)
  {
    targPos11 = -125;   targPos21 = targPos11;   targPos31 = targPos11;   targPos41 = targPos11;
    targPos12 = -450;   targPos22 = targPos12;   targPos32 = targPos12;   targPos42 = targPos12;
    targPos13 =  900;   targPos23 = targPos13;   targPos33 = targPos13;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }

    else
    {
      setPrevPos();
      delay(1000);
      phase = 2;
      i = 0;
    }
  }


  else if (phase == 2)
  {
    targPos11 = initPos11;   targPos21 = initPos21;   targPos31 = initPos31;   targPos41 = initPos41;
    targPos12 = initPos12;   targPos22 = initPos22;   targPos32 = initPos32;   targPos42 = initPos42;
    targPos13 = initPos13;   targPos23 = initPos23;   targPos33 = initPos33;   targPos43 = initPos43;

    if (i < ird)
    {
      increment();
      i++;
    }

    else
    {
      wakeUpDone = true;
    }
  }

  updatePos();
}

void stateMachine()
{
  if (radio.available())
  {
    radio.read(&data, sizeof(DATA_STRUCTURE));

    if (data.joystickY >= 639 && data.button1 == 0) // trot_F()
    {
      if (prevState == 1)
      {
        state = 1;
      }

      else if (prevState != 1)
      {
        Stop();
        state = prevState = 1;
      }
    }

    else if (data.joystickY <= 383 && data.button1 == 0) // trot_B()
    {
      if (prevState == 2)
      {
        state = 2;
      }

      else if (prevState != 2)
      {
        Stop();
        state = prevState = 2;
      }
    }

    else if (data.joystickX >= 639 && data.button1 == 0) // trot_R()
    {
      if (prevState == 3)
      {
        state = 3;
      }

      else if (prevState != 3)
      {
        Stop();
        state = prevState = 3;
      }
    }

    else if (data.joystickX <= 383 && data.button1 == 0) // trot_L()
    {
      if (prevState == 4)
      {
        state = 4;
      }

      else if (prevState != 4)
      {
        Stop();
        state = prevState = 4;
      }
    }

    else if (data.joystickB == 0 && data.button1 == 0) // trot_P()
    {
      if (prevState == 5)
      {
        state = 5;
      }

      else if (prevState != 5)
      {
        Stop();
        state = prevState = 5;
      }
    }

    else if (data.joystickZ >= 639 && data.button1 == 0) // turn_R()
    {
      if (prevState == 6)
      {
        state = 6;
      }

      else if (prevState != 6)
      {
        Stop();
        state = prevState = 6;
      }
    }

    else if (data.joystickZ <= 383 && data.button1 == 0) // turn_L()
    {
      if (prevState == 7)
      {
        state = 7;
      }

      else if (prevState != 7)
      {
        Stop();
        state = prevState = 7;
      }
    }

    else if (data.joystickY <= 383 && data.button3 == 0) // look_U()
    {
      if (prevState == 8)
      {
        state = 8;
      }

      else if (prevState != 8)
      {
        Stop();
        state = prevState = 8;
      }
    }

    else if (data.joystickY >= 639 && data.button3 == 0) // look_D()
    {
      if (prevState == 9)
      {
        state = 9;
      }

      else if (prevState != 9)
      {
        Stop();
        state = prevState = 9;
      }
    }

    else if (data.joystickX >= 639 && data.button3 == 0) // roll_R()
    {
      if (prevState == 10)
      {
        state = 10;
      }

      else if (prevState != 10)
      {
        Stop();
        state = prevState = 10;
      }
    }

    else if (data.joystickX <= 383 && data.button3 == 0) // roll_L()
    {
      if (prevState == 11)
      {
        state = 11;
      }

      else if (prevState != 11)
      {
        Stop();
        state = prevState = 11;
      }
    }

    else if (data.toggle1 == 1) // rest()
    {
      if (prevState == 12)
      {
        state = 12;
      }

      else if (prevState != 12)
      {
        Stop();
        state = prevState = 12;
      }
    }

    else // any unexpected command (or none) run "Stop()"
    {
      state = prevState = 0;
    }
  }

}

void trot_F()
{
  moving = true;
  ird = 10; // define in how many increments each "phase" will be divided (the higher the "ird", the slower the movement)

  if (phase == 1)
  {
    // target positions for phase 1
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();       // run increment function until target positions get reached
      i++;
    }
    else
    {
      delay(phaseDelay); // wait a little for it to get there
      setPrevPos();      // store current positions ("currPos") as previous positions ("prevPos") for the next function
      phase = 2;         // and proceed to phase 2
      i = 0;             // reset "i" for the next phase
    }
  }

  else if (phase == 2)
  {
    // target positions for phase 2
    // RR leg FRONT     // RF leg BACK      // LR leg BACK           // LF leg FRONT
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 =   -5;   targPos22 = -395;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 = -105;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg BACK      // RF leg FRONT     // LR leg FRONT          // LF leg BACK
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -395;   targPos22 =   -5;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 = -105;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos(); // if last phase reach its target positions ("targPos")...
      phase = 1;    // ... return to phase 1
      i = 0;
    }
  }

  updatePos();
}

void trot_B()
{
  moving = true;
  ird = 35;

  if (phase == 1)
  {
    // RR leg FRONT     // RF leg UP        // LR leg BACK      // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = -125;   targPos41 = -125;
    targPos12 =   -5;   targPos22 = -505;   targPos32 = -395;   targPos42 = -295;
    targPos13 =  180;   targPos23 =  495;   targPos33 = -105;   targPos43 =  180;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg UP        // RF leg BACK      // LR leg DOWN      // LF leg FRONT
    targPos11 = -125;   targPos21 = -125;   targPos31 = -125;   targPos41 = -125;
    targPos12 = -505;   targPos22 = -395;   targPos32 = -295;   targPos42 =   -5;
    targPos13 =  495;   targPos23 = -105;   targPos33 =  180;   targPos43 =  180;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg BACK      // RF leg DOWN      // LR leg FRONT     // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = -125;   targPos41 = -125;
    targPos12 = -395;   targPos22 = -295;   targPos32 =   -5;   targPos42 = -505;
    targPos13 = -105;   targPos23 =  180;   targPos33 =  180;   targPos43 =  495;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg DOWN      // RF leg FRONT     // LR leg UP        // LF leg BACK
    targPos11 = -125;   targPos21 = -125;   targPos31 = -125;   targPos41 = -125;
    targPos12 = -295;   targPos22 =   -5;   targPos32 = -505;   targPos42 = -395;
    targPos13 =  180;   targPos23 =  180;   targPos33 =  495;   targPos43 = -105;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      delay(phaseDelay);
      setPrevPos();
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void turn_R()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg OUT       // LR leg OUT            // LF leg OUT
    targPos11 =   -5;   targPos21 =   -5;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -250;   targPos22 = -250;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  105;   targPos23 =  105;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg IN        // LR leg IN             // LF leg IN
    targPos11 = -245;   targPos21 = -245;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -310;   targPos22 = -310;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  200;   targPos23 =  200;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void turn_L()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg OUT       // LR leg OUT            // LF leg OUT
    targPos11 =   -5;   targPos21 =   -5;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -250;   targPos22 = -250;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  105;   targPos23 =  105;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg IN        // LR leg IN             // LF leg IN
    targPos11 = -245;   targPos21 = -245;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -310;   targPos22 = -310;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  200;   targPos23 =  200;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void trot_R()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg IN        // LR leg OUT            // LF leg IN
    targPos11 =   -5;   targPos21 = -245;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -250;   targPos22 = -310;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  105;   targPos23 =  200;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg OUT       // LR leg IN             // LF leg OUT
    targPos11 = -245;   targPos21 =   -5;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -310;   targPos22 = -250;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  200;   targPos23 =  105;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void trot_L()
{
  moving = true;
  ird = 8;

  if (phase == 1)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg OUT       // RF leg IN        // LR leg OUT            // LF leg IN
    targPos11 =   -5;   targPos21 = -245;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -250;   targPos22 = -310;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  105;   targPos23 =  200;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg IN        // RF leg OUT       // LR leg IN             // LF leg OUT
    targPos11 = -245;   targPos21 =   -5;   targPos31 = targPos11;   targPos41 = targPos21;
    targPos12 = -310;   targPos22 = -250;   targPos32 = targPos12;   targPos42 = targPos22;
    targPos13 =  200;   targPos23 =  105;   targPos33 = targPos13;   targPos43 = targPos23;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void trot_P()
{
  moving = true;
  ird = 10;
  if (phase == 1)
  {
    // RR leg UP        // RF leg DOWN      // LR leg DOWN           // LF leg UP
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -505;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  495;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 2;
      i = 0;
    }
  }

  else if (phase == 2)
  {
    // RR leg DOWN      // RF leg DOWN      // LR leg DOWN           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 3;
      i = 0;
    }
  }

  else if (phase == 3)
  {
    // RR leg DOWN      // RF leg UP        // LR leg UP             // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -505;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  495;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 4;
      i = 0;
    }
  }

  else if (phase == 4)
  {
    // RR leg DOWN      // RF leg DOWN      // LR leg DOWN           // LF leg DOWN
    targPos11 = -125;   targPos21 = -125;   targPos31 = targPos21;   targPos41 = targPos11;
    targPos12 = -295;   targPos22 = -295;   targPos32 = targPos22;   targPos42 = targPos12;
    targPos13 =  180;   targPos23 =  180;   targPos33 = targPos23;   targPos43 = targPos13;

    if (i < ird)
    {
      increment();
      i++;
    }
    else
    {
      setPrevPos();
      delay(phaseDelay);
      phase = 1;
      i = 0;
    }
  }

  updatePos();
}

void look_U()
{
  moving = true;
  ird = 50;

  targPos11 = -125;   targPos21 =  -85;   targPos31 = targPos11;   targPos41 = targPos21;
  targPos12 = -440;   targPos22 =  -90;   targPos32 = targPos12;   targPos42 = targPos22;
  targPos13 =  145;   targPos23 = -445;   targPos33 = targPos13;   targPos43 = targPos23;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void look_D()
{
  moving = true;
  ird = 50;

  targPos11 =  -85;   targPos21 = -125;   targPos31 = targPos11;   targPos41 = targPos21;
  targPos12 =  200;   targPos22 = -180;   targPos32 = targPos12;   targPos42 = targPos22;
  targPos13 = -480;   targPos23 =  270;   targPos33 = targPos13;   targPos43 = targPos23;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void roll_R()
{
  moving = true;
  ird = 40;

  targPos11 =  185;   targPos21 = targPos11;   targPos31 = -385;   targPos41 = targPos31;
  targPos12 = -395;   targPos22 = targPos12;   targPos32 = -170;   targPos42 = targPos32;
  targPos13 =  335;   targPos23 = targPos13;   targPos33 =  -35;   targPos43 = targPos33;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void roll_L()
{
  moving = true;
  ird = 40;

  targPos11 = -385;   targPos21 = targPos11;   targPos31 =  185;   targPos41 = targPos31;
  targPos12 = -170;   targPos22 = targPos12;   targPos32 = -395;   targPos42 = targPos32;
  targPos13 =  -35;   targPos23 = targPos13;   targPos33 =  335;   targPos43 = targPos33;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void rest()
{
  moving = true;
  ird = 100;

  targPos11 = -125;   targPos21 = targPos11;   targPos31 = targPos11;   targPos41 = targPos11;
  targPos12 = -450;   targPos22 = targPos12;   targPos32 = targPos12;   targPos42 = targPos12;
  targPos13 =  900;   targPos23 = targPos13;   targPos33 = targPos13;   targPos43 = targPos13;

  if (i < ird)
  {
    increment();
    i++;
  }

  updatePos();
}

void increment() // divides the difference between target positions ("targPos") and previous positions ("prevPos") in small increments
{
  currPos11 += (targPos11 - prevPos11) / (ird);
  currPos12 += (targPos12 - prevPos12) / (ird);
  currPos13 += (targPos13 - prevPos13) / (ird);
  currPos21 += (targPos21 - prevPos21) / (ird);
  currPos22 += (targPos22 - prevPos22) / (ird);
  currPos23 += (targPos23 - prevPos23) / (ird);
  currPos31 += (targPos31 - prevPos31) / (ird);
  currPos32 += (targPos32 - prevPos32) / (ird);
  currPos33 += (targPos33 - prevPos33) / (ird);
  currPos41 += (targPos41 - prevPos41) / (ird);
  currPos42 += (targPos42 - prevPos42) / (ird);
  currPos43 += (targPos43 - prevPos43) / (ird);
}

void setPrevPos() // set the current positions ("currPos") as previous positions ("prevPos") for the next function
{
  prevPos11 = currPos11;
  prevPos12 = currPos12;
  prevPos13 = currPos13;
  prevPos21 = currPos21;
  prevPos22 = currPos22;
  prevPos23 = currPos23;
  prevPos31 = currPos31;
  prevPos32 = currPos32;
  prevPos33 = currPos33;
  prevPos41 = currPos41;
  prevPos42 = currPos42;
  prevPos43 = currPos43;
}

void updatePos() // move to new calculated positions
{
  myLSS11.move(currPos11 - offset11);
  myLSS12.move(currPos12 - offset12);
  myLSS13.move(currPos13 - offset13);
  myLSS21.move(currPos21 - offset21);
  myLSS22.move(currPos22 - offset22);
  myLSS23.move(currPos23 - offset23);
  myLSS31.move(currPos31 - offset31);
  myLSS32.move(currPos32 - offset32);
  myLSS33.move(currPos33 - offset33);
  myLSS41.move(currPos41 - offset41);
  myLSS42.move(currPos42 - offset42);
  myLSS43.move(currPos43 - offset43);
}

void Stop() // runs between one function and other to make sure we start from "common" positions (i.e., initial positions)
{
  ird = 25;

  if (moving == true)
  {
    setPrevPos();

    targPos11 = initPos11;   targPos21 = initPos21;   targPos31 = initPos31;   targPos41 = initPos41;
    targPos12 = initPos12;   targPos22 = initPos22;   targPos32 = initPos32;   targPos42 = initPos42;
    targPos13 = initPos13;   targPos23 = initPos23;   targPos33 = initPos33;   targPos43 = initPos43;

    for (int s = 0; s <= ird; s++)
    {
      increment();
      updatePos();
    }

    moving = false; // once "Stop()" reaches its target positions ("targPos")
  }

  else if (moving == false)
  {
    setPrevPos();
    i = 0;     // reset "increment()" counter for next function
    phase = 1; // reset phase indicator for next function
  }
}
