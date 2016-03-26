#include "nav.h"

#include <sstream>
#include <vector>

void setup() {
    Serial.begin(9600);
}

void loop() {
    std::vector<usonic> sensors({
        usonic(52, 53),
        usonic(50, 51),
        usonic(48, 49),
        usonic(46, 47),
        usonic(44, 45),
        usonic(42, 43)
    });

    std::vector<pid> pids({
        pid(& sensors[fr].input, 1, 1, 0, 0, DIRECT),
        pid(& sensors[fl].input, 1, 1, 0, 0, DIRECT),
        pid(& sensors[fc].input, 1, 1, 0, 0, DIRECT),
        pid(& sensors[rl].input, 1, 1, 0, 0, DIRECT),
        pid(& sensors[rr].input, 1, 1, 0, 0, DIRECT),
        pid(& sensors[rc].input, 1, 1, 0, 0, DIRECT)
    });

    std::vector<servo> servos({ servo(2, 90, 180), servo(3, 0, 90) });

    std::ostringstream debug;

    debug << "\ninput  ";
    for (auto & sensor : sensors) debug << sensor.ping() << " ";

    debug << "\noutput "; 
    for (auto & pid : pids) debug << pid.compute() << " ";

    Serial.println("debug:");
    Serial.println(debug.str().c_str());

    delay(500);
}
