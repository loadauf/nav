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
#define CALIBRATING 1
#define PRINT_DEBUG 1

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

//    bluetooth(int txd, int rxd, int cts, int rts, int mode) : 
Adafruit_BluefruitLE_UART blueto oth_sensor(Serial2, 22);

std::vector<usonic> forward_sensors({
    usonic(fl, 52, 53, 200),
    usonic(fr, 50, 51, 200),
    usonic(fc, 48, 49)
});

std::vector<control_var> forward_control({
    control_var(1, 50, 0),
    control_var(1, 50, 0),
    control_var(1, 50, 0)
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

control_var heading_control(1, 0, 1);
pid_var heading_pid_vars(1, 0, 0);
PID heading_pid(&heading_control.input, &heading_control.output, &heading_control.setpoint, heading_pid_vars.kp, heading_pid_vars.ki, heading_pid_vars.kd, REVERSE);

std::vector<servo> servos({
    servo(2, 50, 1),
    servo(3, 50, -1)
});

float offset = 0;
float phone_heading = 0;  

void wait_for_bluetooth_response(char const * response) {
   String recieved;  
   while (recieved != response) {
        bluetooth_sensor.println("AT+BLEUARTRX");
        bluetooth_sensor.readline();
        
        if ((strcmp(bluetooth_sensor.buffer, "OK") != 0)) {
            recieved = bluetooth_sensor.buffer;
        } 
    }    
}

bool send_bluetooth_command(char const * command) {
    bluetooth_sensor.print("AT+BLEUARTTX=");
    bluetooth_sensor.println(command);

    // check response stastus
    if (! bluetooth_sensor.waitForOK() ) {
        return false;
    }

    return true;
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    
    // setup servos
    servos[fl].attach();
    servos[fr].attach();

    // setup pid controllers
    for (auto & pid : forward_pids) { 
        pid.SetMode(AUTOMATIC);
        pid.SetOutputLimits(0, 15);
    }

    heading_pid.SetMode(AUTOMATIC);
    heading_pid.SetOutputLimits(0, 15);
    
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

    Serial.println("Connected");

#if CALIBRATING 
    delay(1000);
            
    send_bluetooth_command("calibrating");
    wait_for_bluetooth_response("calibrate");
    Serial.println("\nCalibrating .... ");
         
    int const calibration_pin = 31;
    float sum = 0;
    float maximum = 0;
    int tries = 100;
    int num_values = 1;

    // take maximum reading
    for (int i = 0;  i < tries ; i++) {
        bluetooth_sensor.println("AT+BLEUARTRX");
        bluetooth_sensor.readline();

        if ((strcmp(bluetooth_sensor.buffer, "OK") != 0)) {
            // Some data was found, its in the buffer
            String heading_string = bluetooth_sensor.buffer;
            float zero_buffer = heading_string.toFloat();
    
            if (zero_buffer > 0 && zero_buffer < 360) {
                sum += zero_buffer;
                num_values++;
            }
        }
     }

     float average = sum / num_values;

     sensors_event_t mag_event;
     sensors_vec_t orientation;

     mag.getEvent(&mag_event);
     if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
         float corrected_heading = 360 - orientation.heading;
         offset = average - corrected_heading;

         Serial.print("\noffset ");
         Serial.print(offset); 
     } 

     send_bluetooth_command("done");
     Serial.println("\nCalibrating done");
     
     Serial.print("\naverage ");
     Serial.print(average);

     Serial.print("\nsum ");
     Serial.print(sum);

     Serial.print("\nvalues ");
     Serial.print(num_values);
    
     delay(1000);
#endif
     wait_for_bluetooth_response("start");
     Serial.println("Starting ...");
}

bool stopped = false;
void loop() {
    
//    BLUETOOTH HEADING   
//    Check for incoming characters from Bluefruit
    bluetooth_sensor.println("AT+BLEUARTRX");
    bluetooth_sensor.readline();

    if ((strcmp(bluetooth_sensor.buffer, "OK") != 0)) {
        // Some data was found, its in the buffer
        String heading_string = bluetooth_sensor.buffer;
        Serial.print("\nbuffer ");
        Serial.print(bluetooth_sensor.buffer);
        if (heading_string == "stop") {
            stopped = true;
        }
        else if (heading_string == "start") {
            stopped = false;
        }
        
        float zero_buffer = heading_string.toFloat();
        if (zero_buffer) {
            phone_heading = zero_buffer;
        }
    
    }
    
    Serial.print("\nphone_heading ");
    Serial.print(phone_heading);
    
    bool boundary_swap = false;
    bool make_turn = false;
    float heading_difference = 0;
    float abs_heading_difference = 0;
    float const tolerance_angle = 20;

    sensors_event_t mag_event;
    sensors_vec_t orientation;
    
    // IMU HEADING
    mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
        float corrected_heading = 360 - orientation.heading;

        Serial.print("\norientation_heading ");
        Serial.print(orientation.heading);  

        Serial.print("\ncorrected_heading ");
        Serial.print(corrected_heading);  
        
        heading_difference = phone_heading - corrected_heading - offset; 
        Serial.print("\nheading_difference ");
        Serial.print(heading_difference);
         
        abs_heading_difference = abs(heading_difference);
        Serial.print("\nabs_heading_difference ");
        Serial.print(abs_heading_difference);
        
        if (abs_heading_difference > tolerance_angle) {
            make_turn = true;
            Serial.print("\nmake_turn ");
            Serial.print(make_turn);
        }
        
        if (abs_heading_difference > 180) {
            abs_heading_difference = 360 - abs_heading_difference;
            boundary_swap = true;
            Serial.print("\nboundary_swap ");
            Serial.print(boundary_swap);
        }
        
        heading_control.input = abs_heading_difference;   
    }

//    OBSTACLE CONTROL
//    filter zeros from input to prevent jerkiness
    for (auto & sensor : forward_sensors) {
        double reading = sensor.handle.ping_cm();
        if (reading) 
            forward_control[sensor.face].input = reading;
    }

    for (auto & pid : forward_pids) 
        pid.Compute();

//    finds sensor with obstacle within setpoint distance
//    else uses values from front sensor

    bool obstacle_found = false;
    auto obstacle_control = std::find_if(forward_control.begin(), forward_control.end(), [&](control_var & control) {
        obstacle_found = control.input < control.setpoint;
        return obstacle_found;
    });

    if (obstacle_control == forward_control.end()) 
        *obstacle_control = forward_control[fl];

    heading_pid.Compute();
    
    int left_servo_speed = 0; 
    int right_servo_speed = 0;

    if (!bluetooth_sensor.isConnected()) {
//stopped = true;
    }

    Serial.print("\nstopped ");
    Serial.print(stopped);
    
    if (!stopped) {
        left_servo_speed = 15;
        right_servo_speed = 15;
        if (make_turn) {  
            if (!boundary_swap) {
                if (heading_difference > 0) {
                    right_servo_speed += 10;
                }
                else {    
                    left_servo_speed += 10;
                }
            }
            else {
                 if (heading_difference > 0) {
                    left_servo_speed += 10;
                }
                else {    
                    right_servo_speed += 10;
                }
            }
        }

        if (obstacle_found) {
            left_servo_speed = obstacle_control->output;
            right_servo_speed = obstacle_control->output;
        }
    }

    servos[fl].write(left_servo_speed); 
    servos[fr].write(right_servo_speed);

//    DEBUG VOMIT
#if PRINT_DEBUG     
    Serial.print("\nleft ");
    Serial.print(left_servo_speed);

    Serial.print("\nright ");
    Serial.println(right_servo_speed);    

    Serial.print("\n\ninputs  ");
    for (auto & control : forward_control) { Serial.print(control.input); Serial.print(" "); }

    Serial.print("\noutputs  ");
    for (auto & control : forward_control) { Serial.print(control.output); Serial.print(" "); }

    Serial.print("\nangles  ");
    for (auto & servo : servos) { Serial.print(servo.angle); Serial.print(" "); }

    Serial.print("\nobstacle_control ");
    Serial.print(obstacle_control->output); 

    Serial.print("\nheading_input ");
    Serial.print(heading_control.input);
    
    Serial.print("\nheading_control ");
    Serial.print(heading_control.output); 
 #endif   

}
