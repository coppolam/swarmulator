#ifndef KF_H
#define KF_H


#include <vector>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "parameters.h"

using namespace Eigen;

class kf{
private:
	int n,m;
	float dt;

	MatrixXd Xp; // state prediction
	MatrixXd Zp; // measurement prediction
	MatrixXd H;  // jacobian of the measure wrt X
	MatrixXd A;  // state

public:

	MatrixXd X;  // state
	MatrixXd P;  // state covariance matrix
	MatrixXd Q;  // process covariance noise
	MatrixXd R;  // measurement covariance noise

	kf(const int& a, const int&b, const int&c);
	~kf(){};

	void reset();
	void predict();
	void update(MatrixXd& y);
};


#endif /*KF_H*/