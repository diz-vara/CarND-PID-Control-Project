#ifndef PID_H
#define PID_H

class PID {
private:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  //increments
  double dKp;
  double dKi;
  double dKd;

  //staste
  unsigned long idx;
  double prev_cte;
  double sum_cte;

public:

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  double UpdateError(double cte);
  void setKp(double _Kp) {Kp = _Kp;}
  double getKp() const {return Kp;}
  double updateKp(double _dKp) { Kp = Kp + _dKp; return Kp;}

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
