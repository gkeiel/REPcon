#include "REPcon.h"
#include <math.h>


REP::REP(float k_p, float k_r, float T_s, float w_0, float w_c, int n)
{
    // set parameters
    this->T_s = (T_s > 0.0f) ? T_s : 0.01f;
    this->w_0 = (w_0 > 0.0f) ? w_0 : 0.0f;
    this->n   = (n > 0) ? n : 1;

    // set repetitive controller gains
    this->k_p = k_p;
    this->k_r = k_r;
    
    // set pole for LP filter
    this->w_c = (w_c > 0.0f) ? w_c : 1000.0f;

    update_coefficients();
    
    // set default values
    u_max =  255.0f;
    u_min = -255.0f;    
    
    // reset method
    reset();
}

// compute
float REP::compute(float r, float y)
{
    // error
    float e = r -y;

    // proportional control action
    float P = k_p*e;

    // repetitive control action
    n_counter++;
    if (n_counter >= n)
    {
        n_counter = 0;
        R = k_r*(e +LPF(e_buffer[i]));

        // update buffer and index
        e_buffer[i] = e;
        i++;
        
        if (i >= N_eff) i = 0;
    }
    
    // control signal
    float u = P +R;

    // saturation
    if (u > u_max) u = u_max;
    if (u < u_min) u = u_min;
    
    return u;
}

// low-pass filter
float REP::LPF(float x)
{
    float y = b*x +b*x1 -a*y1;
    x1 = x;
    y1 = y;
    return y;
}

void REP::update_coefficients()
{
    // gain correction
    k_c = sqrtf(w_0*w_0 +w_c*w_c)/w_c;

    // Tustin discretization coefficients
    b = k_c*w_c*T_s/(2.0f +w_c*T_s);
    a = (-2.0f +w_c*T_s)/(2.0f +w_c*T_s);

    // effective delay
    tau = (2.0f*3.14159265359f -atanf(w_0/w_c))/w_0;

    // buffer size, adjusted by max if N_eff > REP_MAX_BUF
    N_eff = round(tau/(T_s*n));
    if (N_eff > REP_MAX_BUF)
    {
        n     = (int)ceilf(tau/(T_s*REP_MAX_BUF));
        N_eff = roundf(tau/(T_s*n));
    }

    // validate buffer size
    if (N_eff < 1 || N_eff > REP_MAX_BUF) N_eff = 0;
}

// reset
void REP::reset()
{
    // reset filter states
    x1 = 0.0f;
    y1 = 0.0f;

    // reset repetitive action
    R = 0.0f;

    // reset counters
    i = 0;
    n_counter = 0;

    // clear buffer
    for (int i = 0; i < REP_MAX_BUF; i++) e_buffer[i] = 0.0f;
}

// set methods
void REP::set_tuning(float k_p, float k_r)
{   
    if (k_p < 0.0f || k_r < 0.0f) return;
    this->k_p = k_p;
    this->k_r = k_r;
}
void REP::set_limit(float u_min, float u_max)
{
    if (u_min >= u_max) return;
    this->u_min = u_min;
    this->u_max = u_max;
}
void REP::set_filter(float w_c)
{
    if (w_c <= 0.0f) return;
    this->w_c = w_c;
    update_coefficients();
    reset();
}

// get methods
float REP::get_kp() { return k_p; }
float REP::get_kr() { return k_r; }
int REP::get_samples() { return N_eff; }