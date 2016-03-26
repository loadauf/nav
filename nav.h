#pragma once

// arduino
#include <Servo.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <StandardCplusplus.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <utility.h>

// stl
#include <algorithm>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

// note: all distances are in cm

enum face { fl, fr, fc, rl, rr, rc };

struct usonic {
    usonic(int trigger, int echo) :
        sensor(trigger, echo),
        trigger(trigger),
        echo(echo),
        input(0)
    {}

    double ping() {
        input = sensor.ping_cm();
        return input;
    }

    int const trigger;
    int const echo;
    double input;
    NewPing sensor;
};

struct pid {
    pid (double * input, double target, double k_p, double k_i, double k_d, int direction) :
        output(0),
        setpoint(target),
        controller(input, & output, & setpoint, k_p, k_i, k_d, direction) 
    {
        controller.SetMode(AUTOMATIC);
    }

    double compute() {
        controller.Compute();
        return output;
    }

    double output;
    double setpoint;
    PID controller;
};

struct servo {
    servo(int pin, int min, int max) :
        pin(pin),
        min(min),
        max(max)
    {
        controller.attach(pin, 1000, 2000);
    }
       
    int const pin;
    int const min;
    int const max;
    Servo controller;
};
