#include "PID.h"

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
    sum_err = 0;
    diff_err = 0;
    curr_err = 0;
    prev_err = 0;
}

void PID::UpdateError(double cte) {
    curr_err = cte;
    sum_err += cte;
    diff_err = curr_err - prev_err;
    prev_err = cte;
    return;
}

double PID::TotalError() {
    return -1. * (Kp * curr_err + Kd * diff_err + Ki * sum_err);
}

