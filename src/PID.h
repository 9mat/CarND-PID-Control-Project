#ifndef PID_H
#define PID_H

#include <vector>
#include <ctime>

class PID {
private:
  double p_error, i_error, d_error, last_cte;

protected:
  double Kp, Ki, Kd;  
  double sse;
  int count;

  // twiddle parameters
  double dKp, dKi, dKd;

  // best mean squared errors so far
  double best_mse;


  // current parameter being twiddled
  // 0 is Kp, 1 is Ki and 2 is Kd
  int cur_param;

  // current direction of twiddle
  // true is positive twiddle and false if negative twiddle
  bool cur_twiddle_dir;

  // if current run is the first run (i.e. no twiddling yet)
  bool first_time;

public:
  /*
  * Errors
  */

  /*
  * Constructor
  */
  PID(double kp, double ki, double kd);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError() const;

  void twiddle();

  void reset_twiddle();

  double get_twiddle_error() const;

  // get the reference of the parameter from the id
  double& get_param_ref(int param_id);

  // get the reference of the twiddle parameter from the id
  double& get_dparam_ref(int param_id);

};


#endif /* PID_H */
