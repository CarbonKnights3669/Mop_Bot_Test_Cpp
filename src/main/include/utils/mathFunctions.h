#pragma once
#include <math.h>
#include <complex>

using namespace std;

namespace mf{
    // wrap input to between -pi and pi
    void wrap(double &angle){
        while (angle > M_PI){
            angle -= M_PI*2;
        }
        while (angle < -M_PI){
            angle += M_PI*2;
        }
    }

    // wrap input to between -0.5 and 0.5
    void wrapTurns(double &turns) {
        while (turns > 0.5){
            turns -= 1;
        }
        while (turns < -0.5){
            turns += 1;
        }
    }

    // wrap input to between -180 and 180
    void wrapDeg(double &angle) {
        while (angle > 180){
            angle -= 360;
        }
        while (angle < -180){
            angle += 360;
        }
    }

    // get the projection of a onto b
    double GetProjectionMagnitude(complex<double> a, complex<double> b) {
        // make b a unit vector
        b /= abs(b);
        return a.real()*b.real() + a.imag()*b.imag();
    }
}