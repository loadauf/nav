#include "nav.h"

#include <NewPing.h>
#include <Servo.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <StandardCplusplus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

#include <cmath>
#include <algorithm>
#include <vector>

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define BUFSIZE 128   // Size of the read buffer for incoming data
#define VERBOSE_MODE true 

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

//    bluetooth(int txd, int rxd, int cts, int rts, int mode) : 
Adafruit_BluefruitLE_UART bluetooth_sensor(Serial2, 22);

std::vector<usonic> forward_sensors({
    usonic(fl, 52, 53, 200),
    usonic(fr, 50, 51, 200),
    usonic(fc, 48, 49)
});

std::vector<control_var> forward_control({
    control_var(1, 30, 0),
    control_var(1, 30, 0),
    control_var(1, 30, 0)
});

std::vector<pid_var> forward_pid_vars({
    pid_var(1, 0, 0),
    pid_var(1, 0, 0),
    pid_var(1, 0, 0)
}); 

PID forward_pids[] = {
    PID(&forward_control[fl].input, &forward_control[fl].output, &forward_control[fl].setpoint, forward_pid_vars[fl].kp, forward_pid_vars[fl].ki, forward_pid_vars[fl].kd, REVERSE),
    PID(&forward_control[fr].input, &forward_control[fr].output, &forward_control[fr].setpoint, forward_pid_vars[fr].kp, forward_pid_vars[fr].ki, forward_pid_vars[fr].kd, REVERSE),
    PID(&forward_control[fc].input, &forward_control[fc].output, &forward_control[fc].setpoint, forward_pid_vars[fc].kp, forward_pid_vars[fc].ki, forward_pid_vars[fc].kd, REVERSE)
};

control_var heading_control(1, 1, 1);
pid_var heading_pid_vars(0.1, 0, 0);
PID heading_pid(&heading_control.input, &heading_control.output, &heading_control.setpoint, heading_pid_vars.kp, heading_pid_vars.ki, heading_pid_vars.kd, REVERSE);

std::vector<servo> servos({
    servo(2, 50, 1),
    servo(3, 50, -1)
});

bool is_calibrated = false;
float offset = 0;

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(44, OUTPUT);
    pinMode(46, OUTPUT);
    pinMode(48, OUTPUT);
    pinMode(50, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(31, OUTPUT);
    
    // setup servos
    servos[fl].attach();
    servos[fr].attach();

    // setup pid controllers
    for (auto & pid : forward_pids) { 
        pid.SetMode(AUTOMATIC);
        pid.SetOutputLimits(0, 15);
    }

    heading_pid.SetMode(AUTOMATIC);
    heading_pid.SetOutputLimits(0, 30);
    
    // setup magnetometer
    if (!mag.begin()) {
      /* There was a problem detecting the LSM303 ... check your connections */
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1);
    }
    /* Initialise the module */
    Serial.print(F("Initialising the Bluefruit LE module: "));
  
    if ( !bluetooth_sensor.begin(VERBOSE_MODE) ){
       error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    
    Serial.println( F("OK!") );
  
    if ( FACTORYRESET_ENABLE )
    {
      /* Perform a factory reset to make sure everything is in a known state */
      Serial.println(F("Performing a factory reset: "));
      if ( ! bluetooth_sensor.factoryReset() ){
        error(F("Couldn't factory reset"));
      }
    }
  
    /* Disable command echo from Bluefruit */
    bluetooth_sensor.echo(false);
  
    Serial.println("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    bluetooth_sensor.info();
  
    Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
    Serial.println(F("Then Enter characters to send to Bluefruit"));
    Serial.println();
  
    bluetooth_sensor.verbose(false);  // debug info is a little annoying after this point!
  
    /* Wait for connection */
    while (! bluetooth_sensor.isConnected()) {
        delay(500);
    }

    int const calibration_pin = 31;
    Serial.println("Calibrating .... ");
    delay(3000);
    digitalWrite(calibration_pin, HIGH);
 
    float sum = 0;
    int tries = 1000;
    int num_values = 0;
    
    for (int i = 0;  i < tries ; i++) {
        if (!(strcmp(bluetooth_sensor.buffer, "OK") == 0)) {
              // Some data was found, its in the buffer
              String heading_string = bluetooth_sensor.buffer;
              float zero_buffer = heading_string.toFloat();
              if (zero_buffer) {
                  sum += zero_buffer;
              }
         }
     }

     float average = sum / tries;
     
     sensors_event_t mag_event;
     sensors_vec_t orientation;
     
     mag.getEvent(&mag_event);
     if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
         offset = average - orientation.heading; 
         is_calibrated = true;
     } 

     delay(3000);
     digitalWrite(calibration_pin, LOW);
     Serial.println("Calibrating done");
    
}

float phone_heading = 0;
float heading_difference = 0;

float deg_to_rad(float deg) { return deg * 3.14 / 180; }
float rad_to_deg(float rad) { return rad * 180 / 3.14; }

void loop() {

    // OBSTACLE CONTROL
  /*
    // filter zeros from input to prevent jerkiness
    for (auto & sensor : forward_sensors) {
        double reading = sensor.handle.ping_cm();
        if (reading) forward_control[sensor.face].input = reading;
    }

    for (auto & pid : forward_pids) pid.Compute();

    // finds sensor with obstacle within setpoint distance
    // else uses values from front sensor
    auto obstacle_control = std::find_if(forward_control.begin(), forward_control.end(), [&](control_var & control) {
        return control.input < control.setpoint;
    });

    if (obstacle_control == forward_control.end()) *obstacle_control = forward_control[fc];

    int left_servo_speed = obstacle_control->output;
    int right_servo_speed = obstacle_control->output;
*/
    int left_servo_speed = 15;
    int right_servo_speed = 15;
    // BLUETOOTH HEADING
    
    // Check for incoming characters from Bluefruit
    bluetooth_sensor.println("AT+BLEUARTRX");
    bluetooth_sensor.readline();

    sensors_event_t mag_event;
    sensors_vec_t orientation;
    
    if (!(strcmp(bluetooth_sensor.buffer, "OK") == 0)) {
        // no data
        // Some data was found, its in the buffer
        String heading_string = bluetooth_sensor.buffer;

        float zero_buffer = heading_string.toFloat();
        if (zero_buffer) {
            phone_heading = zero_buffer;
            
        }
    }

//    if (!is_calibrated) {
//        offset = average - orientation.heading;
//        is_calibrated = true;
//    }

    bool boundary_swap = false;
    bool make_turn = false;
    float tolerance_angle = 15;
    float abs_heading_difference = 0;
    
    // IMU HEADING
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
        heading_difference = phone_heading + offset - orientation.heading;  
        
        abs_heading_difference = abs(heading_difference);
        
        if (abs_heading_difference > (tolerance_angle)) {
            make_turn = true;
        }
        
        if (abs_heading_difference > 180) {
            abs_heading_difference = 360 - abs_heading_difference;
            boundary_swap = true;
        }
        
        heading_control.input = abs_heading_difference;   
    }
    
    heading_pid.Compute();
    
    // if phone heading is to the right
    Serial.print("\nheading_difference ");
    Serial.print(heading_difference);
    
    Serial.print("\nabs_heading_difference ");
    Serial.print(abs_heading_difference);
    
    Serial.print("\noffset ");
    Serial.print(offset);

    Serial.print("\nboundary_swap ");
    Serial.print(boundary_swap);

    Serial.print("\nmake_turn ");
    Serial.print(make_turn);

    if (make_turn) {  
        if (!boundary_swap) {
            if (heading_difference < 0) {
                right_servo_speed += 10;
            }
            else {    
                left_servo_speed += 10;
            }
        }
        else {
             if (heading_difference < 0) {
                left_servo_speed += 10;
            }
            else {    
                right_servo_speed += 10;
            }
        }
    }
    
    servos[fl].write(left_servo_speed); 
    servos[fr].write(right_servo_speed);

    // DEBUG VOMIT
    
//    Serial.print("\n\ninputs  ");
//    for (auto & control : forward_control) { Serial.print(control.input); Serial.print(" "); }
//
//    Serial.print("\noutputs  ");
//    for (auto & control : forward_control) { Serial.print(control.output); Serial.print(" "); }
//
//    Serial.print("\nangles  ");
//    for (auto & servo : servos) { Serial.print(servo.angle); Serial.print(" "); }
//
//    Serial.print("\nobstacle_control ");
//    Serial.print(obstacle_control->output); 
//
//    Serial.print("\nheading_input ");
//    Serial.print(heading_control.input);
//    
//    Serial.print("\nheading_control ");
//    Serial.print(heading_control.output); 
//    
//    Serial.print("\nleft ");
//    Serial.print(left_servo_speed);      
//    
//    Serial.print("\nright ");
//    Serial.println(right_servo_speed); 

    /* 'orientation' should have valid .heading data now */
    Serial.print("\nheading ");
    Serial.print(orientation.heading);

    Serial.print("\nphone_heading ");
    Serial.print(phone_heading);

    
   
}
