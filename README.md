# REPcon

A lightweight **discrete-time proportional-repetitive (PREP) control library for Arduino**.

The library implements PREP controllers using a **first-order low-pass filter** and a configurable **buffer for past error samples**. It is designed for tracking periodic reference signals and rejecting periodic disturbances.

The library is intended as a practical reference for implementing, testing, and applying repetitive control algorithms in embedded systems, particularly on **8-bit AVR microcontrollers** such as the ATmega328P. Furthermore, the sampling period is configured by the user and must match the actual interval between consecutive calls to the controller.


## PREP controller

The continuous-time proportional-repetitive controller is defined by:

$$C_{PREP}(s) = k_p +\frac{k_r}{1 -Q(s)e^{-\tau s}}$$

where:
- $k_p$ is the proportional gain;
- $k_r$ is the repetitive gain;
- $Q(s)$ is a first-order low-pass filter;
- $\tau$ is the delay associated with the fundamental frequency.

The low-pass filter is defined as:

$$Q(s) = \frac{k_c\omega_c}{s +\omega_c}$$

where $\omega_c$ is the filter cutoff frequency and $k_c$ is a correction factor.

The discrete-time controller is obtained by applying the Tustin discretization to the low-pass filter transfer function.


## Features

- **PREP control**: Implements discrete-time PREP action.
- **Repetitive compensation**: Implements repetitive action at harmonic frequencies of the fundamental frequency.
- **Low-pass filtering**: Reduces the magnitude of infinite gains at harmonic frequencies.
- **Configurable update factor**: Allows adjustment of the repetitive update rate.
- **Parameter configuration**: Provides methods to adjust PREP gains and additional parameters.


## Installation

Download or clone this repository and place the REPcon folder inside the Arduino libraries directory. Alternatively, install the library through the Arduino IDE using **Sketch → Include Library → Add .ZIP Library**.

## Basic usage

Create a REP controller by specifying its proportional gain, repetitive gain, sampling period, and fundamental angular frequency:

```cpp
#include <REPcon.h>

float k_p = 10;  
float k_r = 5;    
float T_s = 0.01f;
float w_0 = 2.0f*PI*50;

REP rep(k_p, k_r, T_s, w_0);
```

The constructor uses a low-pass filter frequency of 1000 and an update factor fixed by default. These parameters can also be configured explicitly:

```cpp
float w_c = 3000.0f;
int n = 2;
REP rep(k_p, k_r, T_s, w_0, w_c, n);
```

To calculate the control signal, call compute() with the reference and measured output:
```cpp
float u = rep.compute(r, y);
```

## Configuration

The controller provides the following methods:

| Method | Description |
|---|---|
| `set_tuning(k_p, k_r)` | Sets the PREP gains. |
| `set_filter(w_c)` | Sets the low-pass filter frequency. |
| `set_limit(u_min, u_max)` | Sets the output saturation limits. |
| `compute(r, y)` | Computes the control signal. |
| `reset()` | Resets the controller's internal states. |
| `get_kp()` | Returns the proportional gain. |
| `get_kr()` | Returns the repetitive gain. |
| `get_samples()` | Returns the number of buffer samples. |


## Support the Project

Future versions may include improvements and additional features. 

If you find this library useful, consider making a voluntary contribution solely to support this project's maintenance and further development. 

[![Support the Project](https://img.shields.io/badge/Support%20the%20Project-Ko--fi-FF5E5B?logo=ko-fi&logoColor=white)](https://ko-fi.com/gkeiel)