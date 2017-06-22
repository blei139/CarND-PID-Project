#ifndef PID_H
#define PID_H

//#include "Eigen/Dense"
#include <vector>
#include <string>

//using Eigen::MatrixXd;
//using Eigen::VectorXd;

class PID {
public:
  /*
  * Errors
  */
  double p_error; //present error
  double i_error; //integral error
  double d_error; //differential error

  /*
  * Coefficients
  */ 
  double Kp; //present error coefficient
  double Ki; //integral error coefficient
  double Kd; //differential error coefficient

  double cte; //cross track error
  double prev_cte; //previous cte
  double int_cte; //integral cte
  double diff_cte; //differential cte

  //MatrixXd p; //1x3 matrix for Kp, Ki, and Kd
  //MatrixXd dp; //1x3 matrix for delta Kp, delta Ki, and delta Kd
  double dKp, dKi, dKd; //delta Kp, delta Ki, and delta Kd

  int n; //error counter for twiddle
  double steering_value;  //steering angle value

  double sum_error; //sum of cte errors
  double best_err; //best average error
  double err; //average error
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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
