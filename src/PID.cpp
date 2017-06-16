#include "PID.h"
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/
enum PIDComponent {
	P_,
	I_,
	D_
};
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
	best_error = 1000000;
	total_error = 0;
	steps = 0;
}

void PID::UpdateError(double cte) {
	cte_sum += cte;

	p_error = cte;
	i_error = cte_sum;
	d_error = cte - cte_previous;

	cte_previous = cte;
	total_error += pow(cte,2);
	steps++;
}

double PID::TotalError() {
	return total_error/(steps);
}

double PID::getSteeringAngle() {
	double steeringAngle = -Kp*p_error - Ki*i_error - Kd*d_error;
	steeringAngle = steeringAngle > 1 ? 1 : steeringAngle;
	steeringAngle = steeringAngle < -1 ? -1 : steeringAngle;
	return steeringAngle;
}

void PID::twiddle() {
	static double dpp = 0.1;
	static double dpi = 0.001;
	static double dpd = 0.1;
	static PIDComponent component = P_;
	static int entryNumber = 1;
	double error = TotalError();

	if (best_error == 1000000) {
		best_error = error;
	}
	steps = 0;
	total_error = 0;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	cte_previous = 0;
	cte_sum = 0;
	switch (component) {
	case P_:
		if (entryNumber == 1) {
			Kp += dpp;
			entryNumber = 2;
		}
		else if (entryNumber == 2) {
			if (error < best_error) {
				best_error = error;
				dpp *= 1.1;
				component = I_;
				entryNumber = 1;
			}
			else {
				Kp -= 2*dpp;
				entryNumber = 3;
			}
		}
		else {
			if (error < best_error) {
				best_error = error;
				dpp *= 1.1;
			}
			else {
				Kp += dpp;
				dpp *= 0.9;
			}
			component = I_;
			entryNumber = 1;
		}
		break;
	case I_:
		if (entryNumber == 1) {
			Ki += dpi;
			entryNumber = 2;
		}
		else if (entryNumber == 2) {
			if (error < best_error) {
				best_error = error;
				dpi *= 1.1;
				component = D_;
				entryNumber = 1;
			}
			else {
				Ki -= 2*dpi;
				entryNumber = 3;
			}
		}
		else {
			if (error < best_error) {
				best_error = error;
				dpi *= 1.1;
			}
			else {
				Ki += dpi;
				dpi *= 0.9;
			}
			component = D_;
			entryNumber = 1;
		}
		break;
	case D_:
		if (entryNumber == 1) {
			Kd += dpd;
			entryNumber = 2;
		}
		else if (entryNumber == 2) {
			if (error < best_error) {
				best_error = error;
				dpd *= 1.1;
				component = P_;
				entryNumber = 1;
			}
			else {
				Kd -= 2*dpd;
				entryNumber = 3;
			}
		}
		else {
			if (error < best_error) {
				best_error = error;
				dpd *= 1.1;
			}
			else {
				Kd += dpd;
				dpd *= 0.9;
			}
			component = P_;
			entryNumber = 1;
		}
		break;
	default:
		break;
	}
}

