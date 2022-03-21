/* V 0.9 - 20/03/2022 - Daniel Desmartins
    Connected to the Relay Port in AgOpenGPS
    If you find any mistakes or have an idea to improove the code, feel free to contact me. N'hésitez pas à me contacter en cas de problème ou si vous avez une idée d'amélioration.
*/

#include <NewTone.h>
#include <mcp_can.h>
#include <SPI.h>

//Pins and variable for speed:
#define PinOutputImpuls 9
#define PULSE_BY_100M 13100

//Variables for CAN bus
#define SPEED_MCP MCP_16MHZ           // Define your speed for MCP : MCP_16MHZ or MCP_8MHZ
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

uint8_t AOG[] = {0x80, 0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };

//Parsing PGN CAN
uint32_t rxId;
uint8_t len = 0;
uint8_t rxBuf[8];

uint8_t currentData[8] = {0x00, 0x01, 0x00, 0x00, 0x10, 0x27, 0x00, 0xFF}; //Set Command Data

enum _set { FULL, RIGHT, LEFT, R_RIGHT, R_LEFT, I_RIGHT, I_LEFT };
enum _section { Section1, Section2, Section3, Section4, Section5, Section6 };

//The variables used for storage
uint8_t relayLo = 0, lastRelay1 = 0, lastRelay2 = 0, lastRelay3 = 0, lastRelay4 = 0, spreaderStatus = 0;

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
    } else { //Field is Open
      if (spreaderOffResetTimer > 10) {
        offLo = 255;
        onLo = 0;
        if (spreaderStatus) {
          setSpreader(FULL); //Close Spreader
          lastTime -= 4000;
        }
      } else {
        offLo = 0;
        
        //Forces left sections that cannot be closed
        if (bitRead(lastLo, Section2) || bitRead(lastLo, Section3)) {
          bitWrite(lastLo, Section2, 0);
          bitWrite(lastLo, Section3, 0);
        } else if (bitRead(relayLo, Section1)) {
          bitWrite(relayLo, Section2, 1);
          bitWrite(relayLo, Section3, 1);
          bitWrite(onLo, Section2, 1);
          bitWrite(onLo, Section3, 1);
        } else if (bitRead(relayLo, Section2)) {          
          if (bitRead(onLo, Section2)) {
            bitWrite(lastLo, Section2, 1);
            bitWrite(offLo, Section2, 1);
            bitWrite(onLo, Section2, 0);
            bitWrite(offLo, Section3, 1);
            bitWrite(onLo, Section3, 0);
          } else {
            bitWrite(relayLo, Section3, 1);
            bitWrite(onLo, Section3, 1);
          }
        } else if (bitRead(relayLo, Section3)) {
          if (bitRead(onLo, Section3)) {
            bitWrite(lastLo, Section3, 1);
            bitWrite(offLo, Section3, 1);
            bitWrite(onLo, Section3, 0);
          }
        }
        
        //Forces right sections that cannot be closed
        if (bitRead(lastLo, Section5) || bitRead(lastLo, Section4)) {
          bitWrite(lastLo, Section5, 0);
          bitWrite(lastLo, Section4, 0);
        } else if (bitRead(relayLo, Section6)) {
          bitWrite(relayLo, Section5, 1);
          bitWrite(relayLo, Section4, 1);
          bitWrite(onLo, Section5, 1);
          bitWrite(onLo, Section4, 1);
        } else if (bitRead(relayLo, Section5)) {          
          if (bitRead(onLo, Section5)) {
            bitWrite(lastLo, Section5, 1);
            bitWrite(offLo, Section5, 1);
            bitWrite(onLo, Section5, 0);
            bitWrite(offLo, Section4, 1);
            bitWrite(onLo, Section4, 0);
          } else {
            bitWrite(relayLo, Section4, 1);
            bitWrite(onLo, Section4, 1);
          }
        } else if (bitRead(relayLo, Section4)) {
          if (bitRead(onLo, Section4)) {
            bitWrite(lastLo, Section4, 1);
            bitWrite(offLo, Section4, 1);
            bitWrite(onLo, Section4, 0);
          }
        }
        
        if (!offLo) {
          //waits for the position sent by AOG to be stabilized. the spreader cannot change state quickly.
          if (lastRelay1 == relayLo) {
            if (lastRelay2 == lastRelay1) {
              if (lastRelay3 == lastRelay2) {
                lastRelay4 = lastRelay3;
              }
              lastRelay3 = lastRelay2;
            }
            lastRelay2 = lastRelay1;
          }
          lastRelay1 = relayLo;
          
          //Open/Close Right Spreader
          if (bitRead(lastRelay4, Section4)) {
            if (!bitRead(spreaderStatus, Section4)) {
              setSpreader(RIGHT);
            }
          } else if (bitRead(spreaderStatus, Section4)) {
            setSpreader(RIGHT);
          }
         
          //Open/Close Left Spreader
          if (bitRead(lastRelay4, Section3)) {
            if (!bitRead(spreaderStatus, Section3)) {
              setSpreader(LEFT);
            }
          } else if (bitRead(spreaderStatus, Section3)) {
            setSpreader(LEFT);
          }
          
          //reduces or increases the right sections
          if (bitRead(lastRelay4, Section6)) {
            if (!bitRead(spreaderStatus, Section6)) setSpreader(I_RIGHT);
          } else if (bitRead(spreaderStatus, Section6)) setSpreader(R_RIGHT);
          if (bitRead(lastRelay4, Section5)) {
            if (!bitRead(spreaderStatus, Section5)) setSpreader(I_RIGHT);
          } else if (bitRead(spreaderStatus, Section5)) setSpreader(R_RIGHT); 
  
          //reduces or increases the left sections
          if (bitRead(lastRelay4, Section1)) {
            if (!bitRead(spreaderStatus, Section1)) setSpreader(I_LEFT);
          } else if (bitRead(spreaderStatus, Section1)) setSpreader(R_LEFT);
          if (bitRead(lastRelay4, Section2)) {
            if (!bitRead(spreaderStatus, Section2)) setSpreader(I_LEFT);
          } else if (bitRead(spreaderStatus, Section2)) setSpreader(R_LEFT);
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
      if (offLo) return; //quickly updates sections
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
    if (rxBuf[0] == 0xA0 && rxBuf[3] == 0x01 /*rxBuf[3] == 0x02*/) {
      switch (rxBuf[1]) {
        case 0xFD: //rxBuf[2] = 0x03            //Close right
        case 0x29: //rxBuf[2] = 0x04
        case 0x28: //rxBuf[2] = 0x04
          bitWrite(spreaderStatus, Section1, 0);
          bitWrite(spreaderStatus, Section2, 0);
          bitWrite(spreaderStatus, Section3, 0);
          break;
        
        case 0x2A: //rxBuf[2] = 0x04             //Close left
        case 0x2B:
        case 0x01:
          bitWrite(spreaderStatus, Section4, 0);
          bitWrite(spreaderStatus, Section5, 0);
          bitWrite(spreaderStatus, Section6, 0);
          break;
        
        case 0x00: //rxBuf[2] = 0x04             //Fully open right
          bitWrite(spreaderStatus, Section1, 1);
          bitWrite(spreaderStatus, Section2, 1);
          bitWrite(spreaderStatus, Section3, 1);
          break;
        
        case 0x04: //rxBuf[2] = 0x04             //Fully open left
          bitWrite(spreaderStatus, Section4, 1);
          bitWrite(spreaderStatus, Section5, 1);
          bitWrite(spreaderStatus, Section6, 1);
          break;
        
        case 0xFF: //rxBuf[2] = 0x03             //Half open right
          bitWrite(spreaderStatus, Section1, 0);
          bitWrite(spreaderStatus, Section2, 1);
          bitWrite(spreaderStatus, Section3, 1);
          break;
        
        case 0xFE: //rxBuf[2] = 0x03             //Minimum right opening
          bitWrite(spreaderStatus, Section1, 0);
          bitWrite(spreaderStatus, Section2, 0);
          bitWrite(spreaderStatus, Section3, 1);
          break;
        
        case 0x03: //rxBuf[2] = 0x04             //Half open left
          bitWrite(spreaderStatus, Section4, 1);
          bitWrite(spreaderStatus, Section5, 1);
          bitWrite(spreaderStatus, Section6, 0);
          break;
        
        case 0x02: //rxBuf[2] = 0x04             //Minimum left opening
          bitWrite(spreaderStatus, Section4, 1);
          bitWrite(spreaderStatus, Section5, 0);
          bitWrite(spreaderStatus, Section6, 0);
          break;
      }
    } else if (rxBuf[0] == 0xA8 && (rxBuf[1] == 0xCC || rxBuf[1] == 0xCD)) {
      Serial.println("Spreader Work A8!"); //It remains to determine the acceptable minimum number of revolutions
      spreaderOffResetTimer = 0;
      offLo = 0;
    }
  }
  /*else if (rxId == 0x9CE69026) { //Manuel bouton actived
    if (rxBuf[0] == 0x00 && rxBuf[1] == 0x02 && rxBuf[4] == 0x10 && rxBuf[5] == 0x27 && rxBuf[7] == 0xFF) {
      if (rxBuf[2] == 0x50 || rxBuf[2] == 0x51 || rxBuf[2] == 0x52 || rxBuf[2] == 0x53 || rxBuf[2] == 0x54 || rxBuf[2] == 0xED || rxBuf[2] == 0xEE) {
      //bitWrite(spreaderStatus, 7, 1);
        Serial.println("Manuel bouton actived!");
      }
    }
  }*/
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
  }
  CAN0.sendMsgBuf(0x1CE69026, 1, 8, currentData);
}
