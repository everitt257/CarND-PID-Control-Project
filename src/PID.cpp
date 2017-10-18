#include "PID.h"
#include "iostream"
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	// Initialize Kp, Ki and Kd coefficients and errors
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

}

void PID::UpdateError(double cte) {
	double pre_cte = p_error;
	p_error = cte;
	i_error += cte;
	d_error = cte - pre_cte;
}

double PID::TotalError() {
	double total = Kp * p_error + Kd * d_error + Ki * i_error;
	//if (total > 1.0) total = 1.0;
	//if (total < -1.0) total = -1.0;
	return total;
}
/*
void PID::Twiddle(double tol, double cte) {
	// optimize Kp, Kd and Ki
	double params[3] = {Kp,Ki,Kd};
	double dp = [1, 1, 1];
	err = cte * cte;
	int it = 0;
	while(Kp+Ki+Kd > tol){
		cout << "Iteration: " << it << " error square: " << err << endl;
		for(int i=0; i < sizeof(params)/sizeof(params[0]); i++) {
			params[i] += dp[i];
		}
	}
}*/