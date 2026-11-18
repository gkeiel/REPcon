#define REP_MAX_BUF 250
#include <REPcon.h>

const float T_s = 0.01f; // 10 ms
float r = 0.0f; // setpoint
float y = 0.0f; // output
float u = 0.0f; // control input

// reference parameters
float t   = 0.0f; // time
uint8_t A = 50;   // amplitude
uint8_t T = 5;    // reference period
uint8_t cont = 0;

// plant parameters
float w_n  = 10.0f; // natural frequency (rad/s)
float zeta = 0.7f; // damping ratio
float v    = 0.0f;

// controller parameters
const float k_p = 10;
const float k_r = 5;
const float w_0 = 2.0f*PI/T;

// REP (default)
//REP rep(k_p, k_r, T_s, w_0);

// REP
const float w_c = 3000.0f;
const int n = 2;
REP rep(k_p, k_r, T_s, w_0, w_c, n);


void reference(){
    t += T_s;

    float tau = fmodf(t+T/4.0, T);
    r         = (4.0f*A/T)*fabsf(tau -T/2.0f) -A;
}

void setup() {
    Serial.begin(115200);
    //rep.set_limit(-1000.0f, 1000.0f); // set saturation limit
}

void loop() {
    static uint32_t previous = 0; 

    if (millis() -previous >= T_s*1000){
        previous += T_s*1000;

        // triangular-wave reference
        reference();

        // REP control signal
        u = rep.compute(r, y);

        // second-order plant simulated output
        v += T_s*(w_n*w_n*(u -y) -2*zeta*w_n*v);
        y += T_s*v;

        // Serial Plotter: r, y, e, u
        if (++cont >= 10)
        {
            Serial.print(r, 2);
            Serial.print(" ");
            Serial.print(y, 2);
            Serial.print(" ");
            Serial.print(r-y, 2);
            Serial.print(" ");
            Serial.println(u, 2);
            cont = 0;
        }
    }
}