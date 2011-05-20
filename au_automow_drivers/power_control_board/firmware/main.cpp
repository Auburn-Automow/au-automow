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

// One-wire required setup.
OneWire oneWireTop(pin_temperatureTop);
OneWire oneWireBot(pin_temperatureBot);
DallasTemperature temperatureTop(&oneWireTop);
DallasTemperature temperatureBot(&oneWireBot);
DeviceAddress topAddress;
DeviceAddress botAddress;

// Metronomes for the LED blink and ROS messages
Metro ledMetro = Metro(500);
Metro msgMetro = Metro(250);

char batteryState = 0;
#define BS_CHARGING 1
#define BS_DISCHARGING 2
#define BS_CRITICAL 3

char stateOfCharge;

bool ledState = LOW;
bool cutterLeftState;
bool cutterRightState;

/**
 *  Function required for the avr_bridge code to work.  A simple put character
 *  command in the ros namespace.
 *
 *  For the purpose of this code, we are using the Arduino's built-in serial
 *  library, hence the Serial.write.
 */
namespace ros {
    int fputc(char c, FILE *f) {
        Serial.write(c); 
        return 0;
    }
}

/**
 * Callback from the ROS avr_bridge subscription to the CutterControl message.
 *
 * Sets the state to the state included in the CutterControl message, and turns
 * the cutters on and off accordingly
 *
 * @param *msg
 *      The incoming message from the avr_bridge node.
 *
 *  @return
 *      Nothing
 */
void cuttercallback(ros::Msg const *msg)
{
    digitalWrite(pin_leftCutterControl,cc_msg.LeftControl);
    digitalWrite(pin_rightCutterControl,cc_msg.RightControl);
    cutterLeftState = HIGH;
    cutterRightState = HIGH;
}

void updateBatteryDisplay(void)
{
    if(ledState == HIGH)
        ledState = LOW;
    else
        ledState = HIGH;
   
    switch(batteryState)
    {
    case BS_CHARGING:
        digitalWrite(pin_ledHigh,ledState);
        digitalWrite(pin_ledMid,LOW);
        digitalWrite(pin_ledLow,LOW);
        break;
    case BS_CRITICAL:
        digitalWrite(pin_ledHigh,LOW);
        digitalWrite(pin_ledMid,LOW);
        digitalWrite(pin_ledLow,ledState);
        break;
    default:
        unsigned char temp = stateOfCharge/20;
        bool a,b,c;
        switch(temp){
            case 5:
               a=1;b=0;c=0;
               break;
            case 4:
               a=1;b=0;c=0;
               break;
            case 3:
               a=1;b=1;c=0;
               break;
            case 2:
               a=0;b=1;c=0;
               break;
            case 1:
               a=0;b=1;c=1;
               break;
        }
        digitalWrite(pin_ledHigh,a);
        digitalWrite(pin_ledMid,b);
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
        temperatureTop.setWaitForConversion(false);
        temperatureTop.requestTemperatures();
    }
    if(!temperatureBot.getAddress(botAddress,0))
    {
        pcb_msg.Temperature2 = 0;
    } else {
        temperatureBot.setResolution(botAddress,9);
        temperatureBot.setWaitForConversion(false);
        temperatureBot.requestTemperatures();
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
   
    // These were derived from measurements.  Probably could be more accurate.
    float ADCVolts = 0.030024*analogRead(pin_voltage) + 0.027057;
    float ADCAmps = 0.32927 * analogRead(pin_current) - 165.12;
  
    if(ADCVolts > 25.5)
    {
        batteryState = BS_CHARGING;
        stateOfCharge = 100; 
    }   
    else if(ADCVolts < 23)
    {
        batteryState = BS_CRITICAL;
        stateOfCharge = (char)(ADCVolts  * 40 - 900); 
    }
    else
    {
        batteryState = BS_DISCHARGING;
        stateOfCharge = (char)(ADCVolts * 40 - 900); 
    }

    pcb_msg.LeftCutterStatus = (digitalRead(pin_leftCutterCheck) ? FALSE : TRUE);
    pcb_msg.RightCutterStatus = (digitalRead(pin_rightCutterCheck) ? FALSE : TRUE);

    if (ledMetro.check() == 1)
    {
        if(cutterLeftState && !pcb_msg.LeftCutterStatus)
        {
            pcb_msg.LeftCutterStatus = FALSE;
            cutterLeftState= LOW;
            digitalWrite(pin_leftCutterControl,LOW);
            digitalWrite(13,HIGH);
        }
        if(cutterRightState&& !pcb_msg.RightCutterStatus)
        {
            pcb_msg.RightCutterStatus = FALSE;
            cutterRightState = LOW;
            digitalWrite(pin_rightCutterControl,LOW);
            digitalWrite(13,LOW);
        }
        updateBatteryDisplay();
    }

    if (msgMetro.check() == 1)
    {
        pcb_msg.Voltage = ADCVolts;
        pcb_msg.Current = ADCAmps;
        pcb_msg.StateofCharge = stateOfCharge;
        pcb_msg.Temperature1 = temperatureTop.getTempCByIndex(0);
        pcb_msg.Temperature2 = temperatureBot.getTempCByIndex(0);
        temperatureTop.requestTemperatures();
        temperatureBot.requestTemperatures();
        node.publish(pcb_node,&pcb_msg);
    }
}
