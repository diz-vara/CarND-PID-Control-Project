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
    idx = 0;
    sum_cte = 0;
    prev_cte = 0;
}

double PID::UpdateError(double cte) {
    sum_cte += cte;
    double correction = -1. * (Kp * cte + Kd * (cte-prev_cte)+Ki * sum_cte); 

    prev_cte = cte;
    return correction;
}

double PID::TotalError() {
}

