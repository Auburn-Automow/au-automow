#include "WProgram.h"
#include <stdio.h>
#include "avr_ros/ros.h"
#include "avr_ros/CutterControl.h"
#include "avr_ros/PowerControl.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Metro.h>

ros::Publisher pcb_node;
ros::Publisher pcb_status;
avr_bridge::PowerControl pcb_msg;
avr_bridge::CutterControl cc_msg;

// Digital IO
#define pin_leftCutterCheck         2
#define pin_rightCutterCheck        3
#define pin_leftCutterControl       4
#define pin_rightCutterControl      5
#define pin_ledHigh                 6
#define pin_ledMid                  7
#define pin_ledLow                  8
// One-Wire Bus Pins
#define pin_temperatureTop          9
#define pin_temperatureBot          10
// Analog Inputs
#define pin_voltage                 0
#define pin_current                 1


OneWire oneWireTop(pin_temperatureTop);
OneWire oneWireBot(pin_temperatureBot);
DallasTemperature temperatureTop(&oneWireTop);
DallasTemperature temperatureBot(&oneWireBot);
DeviceAddress topAddress;
DeviceAddress botAddress;

Metro ledMetro = Metro(500);
Metro msgMetro = Metro(250);

char batteryState = 0;
#define BS_DISCONNECT 0
#define BS_CHARGING 1
#define BS_DISCHARGING 2
#define BS_CRITICAL 3
char stateOfCharge;
bool ledState = LOW;
bool cutterLeftState;
bool cutterRightState;

namespace ros {
    int fputc(char c, FILE *f) {
        Serial.write(c); 
        return 0;
    }
}

void cuttercallback(ros::Msg const *msg)
{
    digitalWrite(pin_leftCutterControl,cc_msg.LeftControl);
    digitalWrite(pin_rightCutterControl,cc_msg.RightControl);
    cutterLeftState = HIGH;
    cutterRightState = HIGH;
}

void updateBatteryDisplay(void)
{
    if (ledState == LOW)
    { 
        ledState = HIGH;
    } else { 
        ledState = LOW;
    }
    switch(batteryState)
    {
        case BS_DISCONNECT:
            digitalWrite(pin_ledMid,LOW);
            digitalWrite(pin_ledHigh,LOW);
            digitalWrite(pin_ledLow,LOW);
            break;
        case BS_CHARGING:
            digitalWrite(pin_ledLow,LOW);
            digitalWrite(pin_ledMid,LOW);
            digitalWrite(pin_ledHigh,ledState);
            break;
        case BS_DISCHARGING:
            
            break;
        case BS_CRITICAL:
            digitalWrite(pin_ledLow,ledState);
            break;
    }
}

void setup()
{
    Serial.begin(57600);
    pinMode(13,OUTPUT);

    // Set up all of the Digital IO pins.
    pinMode(pin_leftCutterCheck,INPUT);
    pinMode(pin_rightCutterCheck,INPUT);
    pinMode(pin_leftCutterControl,OUTPUT);
    pinMode(pin_rightCutterControl,OUTPUT);
    // Turn off the cutters by default
    digitalWrite(pin_leftCutterControl,LOW);
    digitalWrite(pin_rightCutterControl,LOW);
    pinMode(pin_ledHigh,OUTPUT);
    pinMode(pin_ledMid,OUTPUT);
    pinMode(pin_ledLow,OUTPUT);

    temperatureTop.begin();
    temperatureBot.begin();
    
    // Make sure we have temperature sensors, if not, set to something
    // unreasonable. This would be 0 in Alabama.
    if(!temperatureTop.getAddress(topAddress,0))
    {
        pcb_msg.Temperature1 = 0;
    } else {
        temperatureTop.setResolution(topAddress,9);
    }
    if(!temperatureBot.getAddress(botAddress,0))
    {
        pcb_msg.Temperature2 = 0;
    } else {
        temperatureBot.setResolution(botAddress,9);
    }
    pcb_status = node.advertise("PowerControlStatus");
    pcb_node = node.advertise("PowerControl");
    node.subscribe("CutterControl",cuttercallback,&cc_msg);
}

void loop()
{
    for(;;) {
        int c = Serial.read();
        if (c == EOF)
            break;
        node.spin(c);
    }
    ADC_Volts = analogRead(pin_voltage);
    ADC_Amps = analogRead(pin_current);
    
    pcb_msg.Voltage = ADC_Volts/33.57;
    pcb_msg.Current = (ADC_Amps-504)/3.15;
    pcb_msg.StateofCharge = 100;
    pcb_msg.Temperature1 = temperatureTop.getTempC(topAddress);
    pcb_msg.Temperature2 = temperatureBot.getTempC(botAddress);
    pcb_msg.LeftCutterStatus = digitalRead(pin_leftCutterCheck);
    pcb_msg.RightCutterStatus = digitalRead(pin_rightCutterCheck);

    if (ledMetro.check() == 1)
    {
        if(!pcb_msg.LeftCutterStatus && cutterLeftState)
        {
            digitalWrite(pin_leftCutterControl,LOW);
            cutterLeftState = LOW;
            pcb_msg.LeftCutterStatus = LOW;
        }
        if(!pcb_msg.RightCutterStatus && cutterRightState)
        {
            digitalWrite(pin_rightCutterControl,LOW);
            cutterRightState= LOW;
            pcb_msg.RightCutterStatus = LOW;

        }
        updateBatteryDisplay();
    }

    if (msgMetro.check() == 1)
    {
        node.publish(pcb_node,&pcb_msg);
    }
}
