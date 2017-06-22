#include "PID.h"
#include<iostream>
#include<math.h>

using namespace std;
using std::vector;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi) {
	Kp = Kpi;
	Ki = Kii;
	Kd = Kdi;
}

void PID::UpdateError(double cte) {
	diff_cte = cte - prev_cte; //differential cte error
	int_cte += cte; //integral cte error
	p_error = -Kp * cte; //proportional error
	i_error = -Ki * int_cte; //integral error
	d_error = -Kd * diff_cte; //differential error
	prev_cte = cte; //previous cte error 

}

double PID::TotalError() {
	//sum of all errors
	double steering = p_error + d_error + i_error;
	return steering;
}

