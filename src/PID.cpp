#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	cte_previous = 0;
	cte_sum = 0;
}

void PID::UpdateError(double cte) {
	cte_sum += cte;

	p_error = cte;
	i_error = cte_sum;
	d_error = cte - cte_previous;

	cte_previous = cte;
}

double PID::TotalError() {
	double totalError = -Kp*p_error -Ki*i_error -Kd*d_error;
	return totalError;
}

