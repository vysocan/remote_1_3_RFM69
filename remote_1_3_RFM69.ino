// Remote node for RFM69, with voltage divider(A7) and temperature(A6)
// v.1.30
//
// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9)  PWM
//                  +----+
#include "LowPower.h"
#include <SPI.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <avr/eeprom.h> // Global configuration for in chip EEPROM
#define VERSION     130
// Radio
#define NODEID      4
#define NETWORKID   100
#define GATEWAYID   1
#define FREQUENCY   RF69_868MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define KEY         "ABCDABCDABCDABCD" //has to be same 16 characters/bytes on all nodes, not more not less!
#define ENABLE_ATC  //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI    -75
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif
// OHS
#define REG_LEN       21   // size of one conf. element
#define RADIO_REPEAT  6    // repeat sending

// Global variables
int8_t  res;
uint8_t count, radioLength;
uint8_t pos;
uint8_t msg[REG_LEN+1];

// Configuration struct
struct config_t {
  uint16_t version;
  char     reg[REG_LEN * 3]; // Number of elements on this node
} conf; 

// Float conversion 
union u_tag {
  byte  b[4]; 
  float fval;
} u;

// Registration
void send_conf(){ 
  Serial.print(F("Conf"));
  delay(NODEID*100); // Wait some time to avoid contention
  pos = 0; 
  while (pos < sizeof(conf.reg)) {
    msg[0] = 'R'; // Registration flag
    for (uint8_t ii=0; ii < REG_LEN; ii++){ 
      msg[1+ii] = conf.reg[pos+ii];
    }
    Serial.print(F("-"));
    Serial.print(radio.sendWithRetry(GATEWAYID, msg, REG_LEN + 1, RADIO_REPEAT));
    pos+=REG_LEN;
  }
  Serial.println(F(" end"));
}
// Set defaults on first time
void setDefault(){
  conf.version = VERSION;   // Change VERSION to take effect
  conf.reg[0]  = 'S';       // Sensor
  conf.reg[1]  = 'T';       // Temperature
  conf.reg[2]  = 0;         // Local address
  conf.reg[3]  = B00000000; // Default setting
  conf.reg[4]  = B00011110; // Default setting, group=16, disabled
  for (uint8_t ii=0; ii < 17; ii++){ conf.reg[5+ii] = 0;}
  conf.reg[21] = 'S';       // Sensor
  conf.reg[22] = 'V';       // Humidity
  conf.reg[23] = 0;         // Local address
  conf.reg[24] = B00000000; // Default setting
  conf.reg[25] = B00011110; // Default setting, group=16, disabled
  for (uint8_t ii=0; ii < 17; ii++){ conf.reg[26+ii] = 0;}
  conf.reg[42] = 'S';       // Sensor
  conf.reg[43] = 'X';       // TX power level
  conf.reg[44] = 1;         // Local address
  conf.reg[45] = B00000000; // Default setting
  conf.reg[46] = B00011110; // Default setting, group=16, disabled
  for (uint8_t ii=0; ii < 17; ii++){ conf.reg[47+ii] = 0;}
}

void checkRadio(){
  // Look for incomming transmissions
  if (radio.receiveDone()) {
    radioLength = radio.DATALEN; 
    if (radio.ACKRequested()) { 
      delay(5); // wait after receive, we need this delay or gateway will not see ACK!!!
      radio.sendACK();
      Serial.print(F("ACK:"));
    }
    for (uint8_t ii=0; ii < radioLength; ii++){ Serial.print((char)radio.DATA[ii], HEX); Serial.print("-"); }; Serial.println(F("<"));
    if ((char)radio.DATA[0] == 'C') {
      Serial.print(F("C:"));
      Serial.println(radio.DATA[1],HEX);
      delay(200); // give Gateway some time
      if (radio.DATA[1] == 1) send_conf(); // Registration
    }
    if ((char)radio.DATA[0] == 'R') { // Registration
      Serial.print(F("R:"));
      // Replace part of conf string with new paramters.
      pos = 0; 
      while (((conf.reg[pos] != radio.DATA[1]) || (conf.reg[pos+1] != radio.DATA[2]) || (conf.reg[pos+2] != radio.DATA[3])) && (pos < sizeof(conf.reg))) {
        pos += REG_LEN; // size of one conf. element
        Serial.print(F("."));
      }
      if (pos < sizeof(conf.reg)) {
        Serial.println(pos);
        msg[0] = 'R';
        for (uint8_t ii=0; ii < radioLength-1; ii++){
          conf.reg[pos+ii] = radio.DATA[1+ii];
          msg[1+ii]        = radio.DATA[1+ii];
          Serial.print(msg[1+ii], HEX); Serial.print(F("-"));
        }
        // Save it to EEPROM
        conf.version = VERSION;
        eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
        delay(100); // give Gateway some time
        radio.sendWithRetry(GATEWAYID, msg, radioLength); // Send it back for reregistration
      }
    }
  }
}

void setup() {
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower(); //uncomment only for RFM69HW!
  radio.encrypt(KEY);
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  if (conf.version != VERSION) setDefault();
   
  Serial.begin(115200); 
  count = 0;

  delay(2000);
  send_conf(); 

  Serial.print(F("Start"));
  radio.sleep();
}

void loop() {
  count++;
  if (count == 2) {
    Serial.println(F("."));
    count = 0;
    // Temperature 
    u.fval = (((float)analogRead(A6) * 0.003223)-0.5)*100; 
    msg[0] = 'S'; // Sensor
    msg[1] = 'T'; // Temperature
    msg[2] = 0;   // local address
    msg[3] = u.b[0]; msg[4] = u.b[1]; msg[5] = u.b[2]; msg[6] = u.b[3];
    // BATT Voltage 
    u.fval = 0.0064453125 * (float)analogRead(A7); // Voltage divider 2:1
    msg[7] = 'V'; // Voltage
    msg[8] = 0;   // local address
    msg[9] = u.b[0]; msg[10] = u.b[1]; msg[11] = u.b[2]; msg[12] = u.b[3];
    // TX power level
    u.fval = ((float)radio._transmitLevel*3.03125)+3.0; //3~100% , as 0% maid me woried it's off
    msg[13] = 'X'; // TX power level
    msg[14] = 0;   // local address
    msg[15] = u.b[0]; msg[16] = u.b[1]; msg[17] = u.b[2]; msg[18] = u.b[3];
    // Send to GW 
    radio.sendWithRetry(GATEWAYID, msg, 19);
    // Check for queued messages after sleep
    for (uint8_t i=0; i < RADIO_REPEAT; i++){
      checkRadio();
      delay(10);
    }
    // Put radio to sleep
    radio.sleep();
  }
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // Sleep MCU for ~ 8S
}
