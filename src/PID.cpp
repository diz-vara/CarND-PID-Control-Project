#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;

    dKp = Kp / 5.;
    dKi = Ki / 5.;
    dKd = Kd / 5.;

    idx = 0;
    cnt = 0;
    sum_err = 0;
    sum_sqerr = 0;
    diff_err = 0;
    curr_err = 0;
    prev_err = 0;
    best_err = -1.;

    period = 1024;
    bNeg = false;
}

void PID::UpdateError(double cte) {
    curr_err = cte;
    sum_err += cte;
    diff_err = curr_err - prev_err;
    prev_err = cte;
    return;
}

double PID::TotalError() {
    sum_sqerr += curr_err * curr_err;
    if (cnt++ >= period) {
        sum_sqerr = sum_sqerr / period;
        cnt = 0;
        std::cout << "Twiddle update: " << sum_sqerr << " <?> " << best_err << " : ";
        if (best_err < 0) {
            best_err = sum_sqerr;
            Kp += dKp;
        }
        else if (sum_sqerr < best_err) {
            //better:
            best_err = sum_sqerr; 
            bNeg = false;
            dKp *= 1.1;
            Kp += dKp;
            std::cout << "Yes, ";
        }
        else {
            if (!bNeg) {
                bNeg = true;
                Kp -= 2*dKp;
                std::cout << "no, negate, ";
            } 
            else {
                dKp *= 0.9;
                Kp += dKp;
                bNeg = false;
                std::cout << "no, return and decrease, ";
            }

        }
        std::cout << "Kp = " << Kp << ", dKp = " << dKp << std::endl;
        
    }
    return -1. * (Kp * curr_err + Kd * diff_err + Ki * sum_err);
}

