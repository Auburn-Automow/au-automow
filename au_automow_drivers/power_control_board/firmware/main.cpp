#include "WProgram.h"
#include <stdio.h>
#include "avr_ros/ros.h"
#include "avr_ros/CutterControl.h"
#include "avr_ros/PowerControl.h"
#include <OneWire.h>
#include <DallasTemperature.h>

ros::Publisher pcb_node;
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

namespace ros {
    int fputc(char c, FILE *f) {
        Serial.write(c); 
        return 0;
    }
}

void cuttercallback(ros::Msg const *msg)
{
    digitalWrite(13,cc_msg.LeftControl);
    digitalWrite(pin_rightCutterControl,cc_msg.RightControl);
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
    pinMode(pin_ledHigh,OUTPUT);
    pinMode(pin_ledMid,OUTPUT);
    pinMode(pin_ledLow,OUTPUT);

    temperatureTop.begin();
    temperatureBot.begin();

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
    pcb_msg.Voltage = analogRead(pin_voltage);
    pcb_msg.Current = analogRead(pin_current);
    pcb_msg.StateofCharge = 100;
    pcb_msg.Temperature1 = 10;
    pcb_msg.Temperature2 = 10;
    pcb_msg.LeftCutterStatus = digitalRead(pin_leftCutterCheck);
    pcb_msg.RightCutterStatus = digitalRead(pin_rightCutterCheck);
    node.publish(pcb_node,&pcb_msg);

    delay(100);
}
