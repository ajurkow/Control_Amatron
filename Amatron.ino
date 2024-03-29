/* V 1.4 - 25/09/2022 - Daniel Desmartins
    Connected to the Relay Port in AgOpenGPS
    If you find any mistakes or have an idea to improove the code, feel free to contact me. N'hésitez pas à me contacter en cas de problème ou si vous avez une idée d'amélioration.
*/

#include <NewTone.h>
#include <mcp_can.h>
#include <SPI.h>

//Pins and variable for speed:
#define PinOutputImpuls 9
#define PULSE_BY_100M 13080

//Variables for CAN bus
#define SPEED_MCP MCP_8MHZ            // Define your speed for MCP : MCP_16MHZ or MCP_8MHZ
#define CAN0_INT 2                    // Set INT to pin 2
MCP_CAN CAN0(10);                     // Set CS to pin 10

//Variables:
const uint8_t loopTime = 200; //5hz
uint32_t lastTime = loopTime;
uint32_t currentTime = loopTime;

//Comm checks
uint8_t watchdogTimer = 12;      //make sure we are talking to AOG
uint8_t spreaderOffResetTimer = 12;   //make sure the spreader is working
uint8_t serialResetTimer = 0;   //if serial buffer is getting full, empty it

//speed sent as *10
float gpsSpeed = 0, hertz = 0;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0;
int16_t tempHeader = 0;

//show life in AgIO
uint8_t helloAgIO[] = {0x80,0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0x6D };
uint8_t helloCounter=0;

uint8_t AOG[] = {0x80, 0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//Parsing PGN CAN
uint32_t rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

uint8_t currentData[8] = {0x00, 0x01, 0x00, 0x00, 0x10, 0x27, 0x00, 0xFF}; //Set Command Data

enum _set { FULL, RIGHT, LEFT, R_RIGHT, R_LEFT, I_RIGHT, I_LEFT, I_LEFT_FLOW, I_RIGHT_FLOW, R_LEFT_FLOW, R_RIGHT_FLOW };
enum _section { Section1, Section2, Section3, Section4, Section5, Section6 };
bool leftSpecial = false;
bool rightSpecial = false;
bool manuelMode_L = false;
bool manuelMode_R = false;
bool lastManuelMode = false;

//The variables used for storage
uint8_t relayLo = 0, spreaderStatus = 0;
uint8_t leftSection4 = 0, leftSection3 = 0, leftSection2 = 0, leftSection1 = 0, leftSection = 0;
uint8_t rightSection4 = 0, rightSection3 = 0, rightSection2 = 0, rightSection1 = 0, rightSection = 0;

uint8_t onLo = 0, offLo = 255, lastLo = 0;
//End of variables

void setup() {
  delay(200); //wait for IO chips to get ready

  pinMode(CAN0_INT, INPUT);

  Serial.begin(38400);  //set up communication
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB
  }

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, SPEED_MCP) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else {
    Serial.println("Error Initializing MCP2515...");
  }

  CAN0.setMode(MCP_NORMAL);
} //end of setup

void loop() {
  currentTime = millis();
  if (currentTime - lastTime >= loopTime) {  //start timed loop
    lastTime = currentTime;

    //avoid overflow of watchdogTimer:
    if (watchdogTimer++ > 250) watchdogTimer = 12;
    //avoid overflow of spreaderOffResetTimer:
    if (spreaderOffResetTimer++ > 250) spreaderOffResetTimer = 12;

    //clean out serial buffer to prevent buffer overflow:
    if (serialResetTimer++ > 20) {
      while (Serial.available() > 0) Serial.read();
      serialResetTimer = 0;
      noNewTone();
    }

    //emergency off:
    if (watchdogTimer > 10) {
      offLo = 255;
      onLo = 0;
      if (spreaderStatus && watchdogTimer < 12) setSpreader(FULL); //Close Spreader
      
      //show life in AgIO
      if (++helloCounter > 10) {
        Serial.write(helloAgIO,sizeof(helloAgIO));
        Serial.flush();   // flush out buffer
        helloCounter = 0;
      }
    } else { //Field is Open
      if (spreaderOffResetTimer > 10) {
        offLo = 255;
        onLo = 0;
        if (spreaderStatus) {
          setSpreader(FULL); //Close Spreader
          lastTime -= 4000;
        }
      } else if (manuelMode_L || manuelMode_R) {
        onLo = spreaderStatus;
        offLo = 255 - onLo;
        lastManuelMode = true;
      } else if (lastManuelMode) {
        lastManuelMode = false;
        onLo = 0;
        offLo = 255;
        lastTime -= 500;
      } else {
        offLo = 0;
        
        //Forces left sections that cannot be closed
        if (bitRead(lastLo, Section2) || bitRead(lastLo, Section3)) {
          bitClear(lastLo, Section2);
          bitClear(lastLo, Section3);
        } else if (bitRead(relayLo, Section1)) {
          bitSet(relayLo, Section2);
          bitSet(relayLo, Section3);
          bitSet(onLo, Section2);
          bitSet(onLo, Section3);
        } else if (bitRead(relayLo, Section2)) {          
          if (bitRead(onLo, Section2)) {
            bitSet(lastLo, Section2);
            bitSet(offLo, Section2);
            bitClear(onLo, Section2);
            bitSet(offLo, Section3);
            bitClear(onLo, Section3);
          } else {
            bitSet(relayLo, Section3);
            bitSet(onLo, Section3);
          }
        } else if (bitRead(relayLo, Section3)) {
          if (bitRead(onLo, Section3)) {
            bitSet(lastLo, Section3);
            bitSet(offLo, Section3);
            bitClear(onLo, Section3);
          }
        }
        
        //Forces right sections that cannot be closed
        if (bitRead(lastLo, Section5) || bitRead(lastLo, Section4)) {
          bitClear(lastLo, Section5);
          bitClear(lastLo, Section4);
        } else if (bitRead(relayLo, Section6)) {
          bitSet(relayLo, Section5);
          bitSet(relayLo, Section4);
          bitSet(onLo, Section5);
          bitSet(onLo, Section4);
        } else if (bitRead(relayLo, Section5)) {          
          if (bitRead(onLo, Section5)) {
            bitSet(lastLo, Section5);
            bitSet(offLo, Section5);
            bitClear(onLo, Section5);
            bitSet(offLo, Section4);
            bitClear(onLo, Section4);
          } else {
            bitSet(relayLo, Section4);
            bitSet(onLo, Section4);
          }
        } else if (bitRead(relayLo, Section4)) {
          if (bitRead(onLo, Section4)) {
            bitSet(lastLo, Section4);
            bitSet(offLo, Section4);
            bitClear(onLo, Section4);
          }
        }
        
        if (!offLo) {
          //waits for the position sent by AOG to be stabilized. the spreader cannot change state quickly.
          bitWrite(leftSection4, Section1, bitRead(relayLo, Section1));
          bitWrite(leftSection4, Section2, bitRead(relayLo, Section2));
          bitWrite(leftSection4, Section3, bitRead(relayLo, Section3));
          bitWrite(rightSection4, Section4, bitRead(relayLo, Section4));
          bitWrite(rightSection4, Section5, bitRead(relayLo, Section5));
          bitWrite(rightSection4, Section6, bitRead(relayLo, Section6));
          
          if (leftSection3 == leftSection4) {
            if (leftSection2 == leftSection3) {
              if (leftSection1 == leftSection2) {
                leftSection = leftSection1;
              }
              leftSection1 = leftSection2;
            }
            leftSection2 = leftSection3;
          }
          if (rightSection3 == rightSection4) {
            if (rightSection2 == rightSection3) {
              if (rightSection1 == rightSection2) {
                rightSection = rightSection1;
              }
              rightSection1 = rightSection2;
            }
            rightSection2 = rightSection3;
          }
          
          leftSection3 = leftSection4;
          rightSection3 = rightSection4;
          
          //Open/Close Left Spreader
          if (bitRead(leftSection, Section3)) {
            if (!bitRead(spreaderStatus, Section3)) {
              setSpreader(LEFT);
            }
          } else if (bitRead(spreaderStatus, Section3)) {
            setSpreader(LEFT);
          }
          
          //Open/Close Right Spreader
          if (bitRead(rightSection, Section4)) {
            if (!bitRead(spreaderStatus, Section4)) {
              setSpreader(RIGHT);
            }
          } else if (bitRead(spreaderStatus, Section4)) {
            setSpreader(RIGHT);
          }
         
          //reduces or increases the left sections
          if (!leftSpecial) {
            if (bitRead(leftSection, Section1)) {
              if (!bitRead(spreaderStatus, Section1)) setSpreader(I_LEFT);
            } else if (bitRead(spreaderStatus, Section1)) setSpreader(R_LEFT);
            if (bitRead(leftSection, Section2)) {
              if (!bitRead(spreaderStatus, Section2)) setSpreader(I_LEFT);
            } else if (bitRead(spreaderStatus, Section2)) setSpreader(R_LEFT);
          }
          
          //reduces or increases the right sections
          if (!rightSpecial) {
            if (bitRead(rightSection, Section6)) {
              if (!bitRead(spreaderStatus, Section6)) setSpreader(I_RIGHT);
            } else if (bitRead(spreaderStatus, Section6)) setSpreader(R_RIGHT);
            if (bitRead(rightSection, Section5)) {
              if (!bitRead(spreaderStatus, Section5)) setSpreader(I_RIGHT);
            } else if (bitRead(spreaderStatus, Section5)) setSpreader(R_RIGHT); 
          }
        }
      }
      
      //Send to AOG
      AOG[9] = (uint8_t)onLo;
      AOG[10] = (uint8_t)offLo;
      
      //add the checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(AOG) - 1; i++)
      {
        CK_A = (CK_A + AOG[i]);
      }
      AOG[sizeof(AOG) - 1] = CK_A;
      
      Serial.write(AOG, sizeof(AOG));
      Serial.flush();   // flush out buffer
      
      if (offLo && !manuelMode_L && !manuelMode_R) return; //quickly updates sections
    }
  }

  //Reads the position status of the spreader
  if (!digitalRead(CAN0_INT)) readSpreaderStatus();

  // Serial Receive
  //Do we have a match with 0x8081?
  if (Serial.available() > 4 && !isHeaderFound && !isPGNFound)
  {
    uint8_t temp = Serial.read();
    if (tempHeader == 0x80 && temp == 0x81)
    {
      isHeaderFound = true;
      tempHeader = 0;
    }
    else
    {
      tempHeader = temp;     //save for next time
      return;
    }
  }

  //Find Source, PGN, and Length
  if (Serial.available() > 2 && isHeaderFound && !isPGNFound)
  {
    Serial.read(); //The 7F or less
    pgn = Serial.read();
    dataLength = Serial.read();
    isPGNFound = true;
  }

  //The data package
  if (Serial.available() > dataLength && isHeaderFound && isPGNFound)
  {
    if (pgn == 239) // EF Machine Data
    {
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();

      relayLo = Serial.read();          // read relay control from AgOpenGPS
      Serial.read();

      //Bit 13 CRC
      Serial.read();

      //reset watchdog
      watchdogTimer = 0;

      //Reset serial Watchdog
      serialResetTimer = 0;

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }
    else if (pgn == 254) {
      gpsSpeed = ((float)(Serial.read() | Serial.read() << 8 )); // = Speed * 10
      hertz = (gpsSpeed * PULSE_BY_100M) / 60 / 60; //(= pulse/H) / min / s = Hertz
      if (hertz) NewTone(PinOutputImpuls, hertz);
      else noNewTone();

      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();
      Serial.read();

      //Reset serial Watchdog
      serialResetTimer = 0;

      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }
    else { //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn = dataLength = 0;
    }
  }
} //end of main loop

void readSpreaderStatus() {
  CAN0.readMsgBuf(&rxId, &len, rxBuf);
  if (rxId == 0x9CE72690) { //Status Of Spreader
    if (rxBuf[0] == 0xA0 && rxBuf[3] == 0x01 && (rxBuf[2] == 0x03 || rxBuf[2] == 0x04)) {
      switch (rxBuf[1]) {
        case 0xFD: //rxBuf[2] = 0x03            //Close left
        case 0x29: //rxBuf[2] = 0x04
        case 0x28: //rxBuf[2] = 0x04
          bitClear(spreaderStatus, Section1);
          bitClear(spreaderStatus, Section2);
          bitClear(spreaderStatus, Section3);
          leftSpecial = false;
          manuelMode_L = false;
          break;
        case 0x17: //rxBuf[2] = 0x04 //border spreading
        case 0x0A:                   //limit spreading
        case 0x2C:                   //ditch spreading
          bitClear(spreaderStatus, Section1);
          bitClear(spreaderStatus, Section2);
          bitClear(spreaderStatus, Section3);
          leftSpecial = true;
          manuelMode_L = false;
          break;
        
        case 0x2A: //rxBuf[2] = 0x04             //Close right
        case 0x2B:
        case 0x01:
          bitClear(spreaderStatus, Section4);
          bitClear(spreaderStatus, Section5);
          bitClear(spreaderStatus, Section6);
          rightSpecial = false;
          manuelMode_R = false;
          break;
        case 0x14: //rxBuf[2] = 0x04 //border spreading
        case 0x0B:                   //limit spreading
        case 0x2E:                   //ditch spreading
          bitClear(spreaderStatus, Section4);
          bitClear(spreaderStatus, Section5);
          bitClear(spreaderStatus, Section6);
          rightSpecial = true;
          manuelMode_R = false;
          break;
        
        case 0x00: //rxBuf[2] = 0x04             //Fully open left
          bitSet(spreaderStatus, Section1);
          bitSet(spreaderStatus, Section2);
          bitSet(spreaderStatus, Section3);
          leftSpecial = false;
          break;
        case 0x18: //rxBuf[2] = 0x04 //border spreading
        case 0xFC: //rxBuf[2] = 0x03 //limit spreading
        case 0x2D: //rxBuf[2] = 0x04 //ditch spreading
          bitSet(spreaderStatus, Section1);
          bitSet(spreaderStatus, Section2);
          bitSet(spreaderStatus, Section3);
          leftSpecial = true;
          break;
        
        case 0x04: //rxBuf[2] = 0x04             //Fully open right
          bitSet(spreaderStatus, Section4);
          bitSet(spreaderStatus, Section5);
          bitSet(spreaderStatus, Section6);
          rightSpecial = false;
          break;
        case 0x15: //rxBuf[2] = 0x04 //border spreading
        case 0x07:                   //limit spreading
        case 0x2F:                   //ditch spreading
          bitSet(spreaderStatus, Section4);
          bitSet(spreaderStatus, Section5);
          bitSet(spreaderStatus, Section6);
          rightSpecial = true;
          break;
        
        case 0xFF: //rxBuf[2] = 0x03             //Half open left
          bitClear(spreaderStatus, Section1);
          bitSet(spreaderStatus, Section2);
          bitSet(spreaderStatus, Section3);
          break;
        
        case 0xFE: //rxBuf[2] = 0x03             //Minimum left opening
          bitClear(spreaderStatus, Section1);
          bitClear(spreaderStatus, Section2);
          bitSet(spreaderStatus, Section3);
          break;
        
        case 0x03: //rxBuf[2] = 0x04             //Half open right
          bitSet(spreaderStatus, Section4);
          bitSet(spreaderStatus, Section5);
          bitClear(spreaderStatus, Section6);
          break;
        
        case 0x02: //rxBuf[2] = 0x04             //Minimum right opening
          bitSet(spreaderStatus, Section4);
          bitClear(spreaderStatus, Section5);
          bitClear(spreaderStatus, Section6);
          break;
      }
    } else if (rxBuf[0] == 0xA8 && (rxBuf[1] == 0xCC || rxBuf[1] == 0xCD)) {
      //Serial.println("Spreader Work A8!"); //It remains to determine the acceptable minimum number of revolutions
      spreaderOffResetTimer = 0;
      offLo = 0;
    }
  }
  else if (rxId == 0x9CE69026) { //Manuel bouton actived
    if (rxBuf[0] == 0x00 && rxBuf[1] == 0x02 && rxBuf[4] == 0x10 && rxBuf[5] == 0x27 && rxBuf[7] == 0xFF) {
      if (rxBuf[2] == 0x50)
        manuelMode_L = manuelMode_R = true;
      else if (rxBuf[2] == 0xED)
        manuelMode_R = true;
      else if (rxBuf[2] == 0xEE)
        manuelMode_L = true;
    }
  }
}

void setSpreader(uint8_t set) {
  switch (set) {
    case RIGHT:             //Open/Close Right
      currentData[2] = 0xED;
      currentData[3] = 0x13;
      currentData[6] = 0x08;
      break;
    
    case LEFT:              //Open/Close Left
      currentData[2] = 0xEE;
      currentData[3] = 0x13;
      currentData[6] = 0x09;
      break;
    
    case R_RIGHT:           //Reduces Right
      currentData[2] = 0x53;
      currentData[3] = 0x14;
      currentData[6] = 0x10;
      break;
    
    case R_LEFT:            //Reduces Left
      currentData[2] = 0x54;
      currentData[3] = 0x14;
      currentData[6] = 0x11;
      break;
    
    case I_RIGHT:           //Increases Right
      currentData[2] = 0x51;
      currentData[3] = 0x14;
      currentData[6] = 0x0E;
      break;
    
    case I_LEFT:            //Increases Left
      currentData[2] = 0x52;
      currentData[3] = 0x14;
      currentData[6] = 0x0F;
      break;
    
    case FULL:              //Open/Close Full
      currentData[2] = 0x50;
      currentData[3] = 0x14;
      currentData[6] = 0x07;
      break;
    
    case I_LEFT_FLOW:       //Increases Left Flow
      currentData[2] = 0x89;
      currentData[3] = 0x13;
      currentData[6] = 0x02;
      break;
    
    case I_RIGHT_FLOW:      //Increases Right Flow
      currentData[2] = 0x88;
      currentData[3] = 0x13;
      currentData[6] = 0x01;
      break;
    
    case R_LEFT_FLOW:       //Reduces Left Flow
      currentData[2] = 0x8B;
      currentData[3] = 0x13;
      currentData[6] = 0x04;
      break;
    
    case R_RIGHT_FLOW:      //Reduces Right Flow
      currentData[2] = 0x8A;
      currentData[3] = 0x13;
      currentData[6] = 0x03;
      break;
  }
  CAN0.sendMsgBuf(0x1CE69026, 1, 8, currentData);
}
