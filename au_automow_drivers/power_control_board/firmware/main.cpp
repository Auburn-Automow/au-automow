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

OneWire oneWireTop(9);
OneWire oneWireBot(10);
DallasTemperature temperatureTop(&oneWireTop);
DallasTemperature temperatureBot(&oneWireBot);

namespace ros {
    int fputc(char c, FILE *f) {
        Serial.write(c); 
        return 0;
    }
}

void toggle()
{ //toggle an led to debug the program
    static char t=0;
    if (!t ) {
        digitalWrite(13, HIGH);   // set the LED on
        t = 1;
    } else {
        digitalWrite(13, LOW);    // set the LED off
        t = 0;
    }
}

void cuttercallback(ros::Msg const *msg)
{
    toggle();
}

void setup()
{
    Serial.begin(57600);
    pinMode(13,OUTPUT);
   
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
    pcb_msg.Voltage = 24;
    pcb_msg.Current = 10;
    pcb_msg.StateofCharge = 100;
    pcb_msg.Temperature1 = 10;
    pcb_msg.Temperature2 = 10;
    pcb_msg.LeftCutterStatus = false;
    pcb_msg.RightCutterStatus = true;
    node.publish(pcb_node,&pcb_msg);

    delay(100);
}
