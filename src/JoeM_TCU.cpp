#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"
#include <JoeM_TCU.h>
#include <avr/sleep.h>
//#include <avr/interrupt.h>

const int SPI_CS_PIN = 17;  // CANBed V1
const int LED        = 13;
const int REL_Pump   = 11;
const int REL_Cool   = 10;
const int REL_Heat   = 9;
const int REL4       = 8;
boolean ledON        = 1;
MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin
unsigned char msgBuff[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool Cooling_ON = 0;
bool Heating_ON = 0;

uint32_t Timeout = 10000; // 10s Timeout
uint32_t Timeout_Timer = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    pinMode(REL_Pump, OUTPUT);
    pinMode(REL_Cool, OUTPUT);
    pinMode(REL_Heat, OUTPUT);
    pinMode(REL4, OUTPUT);
    //while(!Serial); //wait for Serial
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {            // init can bus : baudrate = 500k
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");

    CAN.init_Mask(0, 0, 0xFFF);                // Init first mask
    CAN.init_Mask(1, 0, 0xFFF);                // Init second mask
    CAN.init_Filt(0, 0, 0x35A);
}

void MCP2515_ISR(){
    //Serial.begin(115200);
}

void loop() {

    if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data coming
        Serial_clear();
        CAN_read();
        CAN_Heartbeat();
        Timeout_Timer = 0;
    } 
    if(!Timeout_Timer){Timeout_Timer = millis() + Timeout;}
    if (millis() > Timeout_Timer){
        Serial_clear();
        //Serial.println("Timeout! No Data");
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

    /*
    if(Serial.availableForWrite()){
        unsigned long canId = CAN.getCanId();
        Serial.print(canId, HEX);
        Serial.print(" ");
        for (int i = 0; i < len; i++) { // print the data
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    */

    digitalToggle(LED);

    if(buf[4] & 0b01000000 || buf[0] & 0b01000000){Cooling_ON = true;}  // enable Cooling if Warning or Alarm is raised
    if(buf[4] & 0b10000000 && buf[0] & 0b10000000){Cooling_ON = false;} // Disable Cooling if both Warning und ALarm are disabled
    if(buf[5] & 0b00000001 || buf[1] & 0b00000001){Heating_ON = true;}  // enable Heating if Warning or Alarm is raised
    if(buf[5] & 0b00000010 && buf[1] & 0b00000010){Heating_ON = false;} // Disable heating if both Warning und ALarm are disabled
    /*
    if(Serial.availableForWrite()){
        Serial.println("Cooling: "+String(Cooling_ON));
        Serial.println("Heating: "+String(Heating_ON));
    }
    */
}

void CAN_Heartbeat(){
    msgBuff[0] = 0x00;
    msgBuff[1] = 0x00;
    msgBuff[2] = 0x00;
    msgBuff[3] = 0x00;
    msgBuff[4] = 0x00;
    msgBuff[5] = 0x00;
    msgBuff[6] = 0x00;
    msgBuff[7] = 0x00;
    CAN.sendMsgBuf(0xBA, 0, 8, msgBuff);            
    //Serial.println("Heartbeat send.");
    delay(2);
}

void OUT_handle(){
    if (Cooling_ON){digitalWrite(REL_Cool, HIGH);digitalWrite(REL_Pump, HIGH);}
    if (Heating_ON){digitalWrite(REL_Heat, HIGH);digitalWrite(REL_Pump, HIGH);}
}

void enterSleepMode(){
    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
    //Serial.println("going to sleep");
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
    detachInterrupt(0);
}