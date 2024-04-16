#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"
#include <JoeM_TCU.h>
#include <avr/sleep.h>
//#include <avr/interrupt.h>

#define SPI_CS_PIN 17
#define LED 13
#define REL_Pump 11
#define REL_Cool 10
#define REL_Heat 9
#define REL4 8
boolean ledON        = 1;
MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin
unsigned char msgBuff[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Reails board is low active
#define ON 0
#define OFF 1

bool Cooling_ON = OFF;
bool Heating_ON = OFF;

byte Debug = 0;

#define Timeout 10000 // 10s Timeout
uint32_t Timeout_Timer = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    pinMode(REL_Pump, OUTPUT);
    pinMode(REL_Cool, OUTPUT);
    pinMode(REL_Heat, OUTPUT);
    pinMode(REL4, OUTPUT);
    digitalWrite(REL_Cool, OFF);
    digitalWrite(REL_Heat, OFF);
    digitalWrite(REL_Pump, OFF);
    digitalWrite(REL4, OFF);
    //while(!Serial); //wait for Serial
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

    CAN.init_Mask(0, 0, 0xFFF);                // Init first mask
    CAN.init_Mask(1, 0, 0xFFF);                // Init second mask
    CAN.init_Filt(0, 0, 0x35A);                // filter for 0x35A only
}

void MCP2515_ISR(){
    //Serial.begin(115200);
}

void loop() {

    if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data is oming
        if(Debug)Serial_clear();
        CAN_read();
        OUT_handle();
        CAN_Heartbeat();
        Timeout_Timer = 0;
    } 
    if(!Timeout_Timer){Timeout_Timer = millis() + Timeout;}
    if (millis() > Timeout_Timer){
        if(Debug){
            Serial_clear();
            Serial.println("Timeout! No Data");
        }
        enterSleepMode();
    }
}

inline void digitalToggle(byte pin){
    digitalWrite(pin, !digitalRead(pin));
}

void Serial_clear(){Serial.write(12);}

void CAN_read(){
    byte len = 0;
    byte buf[8];
    CAN.readMsgBuf(&len, buf);    // read data, len: data length, buf: data buf

    if(Serial.availableForWrite() && Debug){
        unsigned long canId = CAN.getCanId();
        Serial.print(canId, HEX);
        Serial.print(" ");
        for (int i = 0; i < len; i++) { // print the data
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    digitalToggle(LED);

    if(buf[4] & 0b01000000 || buf[0] & 0b01000000){Cooling_ON = ON;}  // enable Cooling if Warning or Alarm is raised
    if(buf[4] & 0b10000000 && buf[0] & 0b10000000){Cooling_ON = OFF;} // Disable Cooling if both Warning und ALarm are disabled
    if(buf[5] & 0b00000001 || buf[1] & 0b00000001){Heating_ON = ON;}  // enable Heating if Warning or Alarm is raised
    if(buf[5] & 0b00000010 && buf[1] & 0b00000010){Heating_ON = OFF;} // Disable heating if both Warning und ALarm are disabled
    
    if(Serial.availableForWrite() && Debug){
        Serial.println("Cooling: "+String(!Cooling_ON));
        Serial.println("Heating: "+String(!Heating_ON));
    }
    
}

void CAN_Heartbeat(){
    // Data "+ 1" to use 0 in BMC to check for nonexisting Data.
    msgBuff[0] = !digitalRead(REL_Pump)+1;
    msgBuff[1] = !digitalRead(REL_Cool)+1;
    msgBuff[2] = !digitalRead(REL_Heat)+1;
    msgBuff[3] = !digitalRead(REL4)+1;
    msgBuff[4] = 0x00;
    msgBuff[5] = 0x00;
    msgBuff[6] = 0x00;
    msgBuff[7] = 0x00;
    CAN.sendMsgBuf(0xBA, 0, 8, msgBuff);         // [ToDo] Change ID?
    if(Debug)Serial.println("Heartbeat send.");
    if(Debug == 2){
        Serial.println(!digitalRead(REL_Pump)+1);
        Serial.println(!digitalRead(REL_Cool)+1);
        Serial.println(!digitalRead(REL_Heat)+1);
        Serial.println(!digitalRead(REL4)+1);        
    }
    delay(2);
}

void OUT_handle(byte AllOFF){
    if(AllOFF){
        digitalWrite(REL_Pump, OFF);
        digitalWrite(REL_Cool, OFF);
        digitalWrite(REL_Heat, OFF);
        digitalWrite(REL4, OFF);
    } else {
        digitalWrite(REL_Pump, ON);           // Pump always on.
        if (Cooling_ON != Heating_ON){          // Check if either cooling or heating is on.
            digitalWrite(REL_Cool, Cooling_ON);
            digitalWrite(REL_Heat, Heating_ON);
        } else {                                // Disable both if both are on. = Error!
            digitalWrite(REL_Cool, OFF);
            digitalWrite(REL_Heat, OFF);
        }
    }
}

void enterSleepMode(){
    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
    OUT_handle(1); // turn everything off
    if(Debug)Serial.println("going to sleep");
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
    detachInterrupt(0);
}