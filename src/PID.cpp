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
    cnt = -50; //to skip first steps at low speed
    sum_err = 0;
    sum_sqerr = 0;
    diff_err = 0;
    curr_err = 0;
    prev_err = 0;
    best_err = -1.;

    pK[0]  = &Kp;   pK[1] = &Ki;   pK[2] = &Kd;
    pdK[0] = &dKp; pdK[1] = &dKi; pdK[2] = &dKd;
    
    period = 400;
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
    //skip straight paths
    if (curr_err * curr_err > 1e-5 ) {
        //to skip initial points
        if (cnt >= 0)
            sum_sqerr += curr_err * curr_err;
        cnt++;
    }
    if (period < 5000 && cnt >= period) {
        if (Twiddle()) //if the result is good
            period = period + period / 4;
    }
    return -1. * (Kp * curr_err + Kd * diff_err + Ki * sum_err);
}

/*
    'Twiddle' function updates PID coefficients
    returns true if new error (mean squared) is less
    than previous min
*/

bool PID::Twiddle() {
    sum_sqerr = sum_sqerr / period;
    cnt = 0;
    double *pk = pK[idx];
    double *pdk = pdK[idx];

    //indicate the  decrease of the error
    bool bResult(false);

    
    if (best_err < 0) {
        best_err = sum_sqerr;
        *pk += *pdk;
        return false;
    }

    std::cout << "Twiddle update[" << idx << "] (" << period << "): " << sum_sqerr << " <?> " << best_err << " : ";
    
    if (sum_sqerr < best_err) {
        //better:
        best_err = sum_sqerr; 
        bNeg = false;
        *pdk *= 1.1;
        idx++;
        bResult = true;
        std::cout << "Yes, ";
    }
    else {
        if (!bNeg) {
            bNeg = true;
            *pk -= 3 * *pdk;
            std::cout << "no, negate, ";
        } 
        else {
            *pk += *pdk;
            *pdk *= 0.9;
            bNeg = false;
            idx++;
            std::cout << "no, return and decrease, ";
        }
    }
    if (idx >= 3) 
        idx = 0;
    *pK[idx] += *pdK[idx];


    std::cout << endl << "Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << " (";
    std::cout << dKp << ", " << dKi << ", " << dKd << ")" << std::endl;
    return bResult;
}

void PID::Restart()
{
    cnt = -100;
    sum_sqerr = 0;
}