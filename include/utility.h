#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Core>
#include <Eigen/Geometry>

void CameraToWorld(double* Rp, double* tp, double* tcwp)
{
	Eigen::Matrix<double,3,1>::ConstMapType R(Rp,3);
	Eigen::Matrix<double,3,1>::ConstMapType t(tp,3);
	Eigen::AngleAxisd rotation_vector(R.norm(), R/R.norm());
	Eigen::Matrix<double,3,1> tcw = -(rotation_vector.toRotationMatrix()).transpose() * t;
	for (int i = 0; i < 3; i++)
	{
		*(tcwp+i) = tcw(i,0);
	}
	return;
}


#endif