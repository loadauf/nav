#pragma once

// arduino
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

enum Face {fl, fr, fc, rl, rr};

struct control_var {
    control_var(double input, double setpoint, double output) :
        input(input),
        setpoint(setpoint),
        output(output) 
    {}

    double input;
    double setpoint;
    double output;
};

struct pid_var {
    pid_var(double kp, double ki, double kd) :
        kp(kp),
        ki(ki),
        kd(kd)
    {}

    double kp;
    double ki;
    double kd;
};

struct usonic {
    usonic(Face face, int trigger, int echo, int max = 500) :
        face(face),
        handle(trigger, echo, max)
    {}

    Face face;
    NewPing handle;
};

struct servo {
    servo (int pin, int max, int orientation = 1) :
        pin(pin),
        orientation(orientation),
        rest_angle(90),
        max_angle(rest_angle + (orientation * max)),
        angle(0)
    {}

    int const pin;
    int const orientation;
    int const rest_angle;
    int const max_angle;
    int angle;
    Servo controller;

    void attach() {
      controller.attach(pin);
    }
    
    void write(int value) {
        int normal_value = value < max_angle? value : max_angle;
        angle = rest_angle + (orientation * normal_value);
        controller.write(angle);
    }
};

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
