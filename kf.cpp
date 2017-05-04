#include "kf.h"

kf::kf(const int& n_states, const int& n_meas, const int& dtime)
{
	n = n_states;
	m = n_meas;
	dt = dtime;

	X.resize(n,1);  // state

	A.resize(n,n);
	P.resize(n,n);
	Q.resize(n,n);
	R.resize(n,n);

};

/* Resets the state voidector and covariance matrices of the EKF */
void kf::reset()
{
	P = MatrixXd::Identity(n,n);  // Make identity matrix
	X.MatrixXd::setZero(n,1); // Initial state
}

/* Perform the prediction step
	PREDICT:
		Predict state
			x_p = f(x);
			A = Jacobian of f(x)
		
		Predict P
			P = A * P * A' + Q;
		
		Predict measure
			z_p = h(x_p)
			H = Jacobian of h(x)
	
*/
void kf::predict() {

	// Fetch dt, dX and A given the current state X and input u
	Xp = A*X;
	Zp = H*Xp;
	P = A * P * A.transpose() + Q;
}

/* Perform the update step
	UPDATE:
		Get Kalman Gain
			P12 = P * H';
			K = P12/(H * P12 + R);
		
		Update x
			x = x_p + K * (z - z_p);
		
		Update P
			P = (eye(numel(x)) - K * H) * P;
*/
void kf::update(matrixXd& Z)
{

	MatrixXd E = H * P * H.transpose() + R;
	MatrixXd K = P * H.transpose() * E.inverse(); // Get kalman gain
	P = P - K * H * P;
	
	MatrixXd err = Z - Zp;
	X = X + K * err;

}