#ifndef REPcon_h
#define REPcon_h
#ifndef REP_MAX_BUF
#define REP_MAX_BUF 250
#endif


class REP {
    public:
        // constructor
        REP(float k_p, float k_r, float T_s, float w_0, float w_c = 1000.0f, int n = 1);

        // configuration
        void set_tuning(float k_p, float k_r);
        void set_limit(float u_min, float u_max);
        void set_filter(float w_c);

        // compute repetitive control signal
        float compute(float r, float y);

        // reset control signals
        void reset();

        // get methods
        float get_kp();
        float get_kr();
        int get_samples();

    private:
        // controller
        float k_p, k_r, T_s, tau, w_0, w_c, k_c;
        float R, e_buffer[REP_MAX_BUF];
        float u_min, u_max;

        // filter
        float LPF(float x), a, b;
        float x1, y1;

        // buffer
        int N_eff, n, i, n_counter;

        void update_coefficients();
};
#endif