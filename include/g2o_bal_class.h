#include <Eigen/Core>
#include "sophus/se3.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include <math.h>

//typedef Eigen::Matrix<double, 9, 1> Vector9d;

class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexCameraBAL(){}
	virtual bool read ( std::istream& in){
		return false;
	}
	virtual bool write ( std::ostream& out) const {
		return false;
	}
	virtual void setToOriginImpl(){}

	virtual void oplusImpl (const double* update)
	{
		Eigen::Matrix<double,6,1> v = _estimate.head<6>();
		Eigen::Matrix<double, 6, 1>::ConstMapType v1(update, 6); // update for pose, trans in front, rat in back, same with sophus 
		Eigen::Matrix<double, 3, 1>::ConstMapType v2(update+6, 3); // update for param
		for (int i = 0; i < 3; i++)
		{
			_estimate(i+6) += v2(i);
		}
		Eigen::Matrix<double,6,1> v3 = (Sophus::SE3::exp(v1) * Sophus::SE3::exp(v)).log(); // need to note, r is in front, t is in back
		for (int i = 0; i < 6; i++)
		{
			_estimate(i) = v3(i);
		}
	}
};


class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexPointBAL(){}

	virtual bool read ( std::istream& in){
		return false;
	}
	virtual bool write ( std::ostream& out) const {
		return false;
	}
	virtual void setToOriginImpl(){}

	virtual void oplusImpl (const double* update)
	{
		Eigen::Vector3d::ConstMapType v(update);
		_estimate += v;
	}

};


class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EdgeObservationBAL(){}

	virtual bool read( std::istream& ){
		return false;
	}

	virtual bool write(std::ostream&) const{
		return false;
	}

	virtual void computeError()
	{
		const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> (vertex(0));
		const VertexPointBAL* point = static_cast<const VertexPointBAL*> (vertex(1));
		Eigen::Matrix<double,9,1> cam_est = cam->estimate();
		Eigen::Matrix<double,3,1> point_est = point->estimate();

		Eigen::Matrix<double,3,1> rotation_part(cam_est(3),cam_est(4),cam_est(5));
		Eigen::Matrix<double,3,1> translation_part(cam_est(0), cam_est(1), cam_est(2));
		Eigen::AngleAxisd rotation_vector(rotation_part.norm(), rotation_part/rotation_part.norm());
		Eigen::Matrix<double,3,1> after_rotation = rotation_vector.matrix() * point_est + translation_part;
		Eigen::Matrix<double,2,1> after_rotation_2d(-after_rotation(0)/after_rotation(2), -after_rotation(1)/after_rotation(2));
		double radius = 1 + cam_est(7)*(after_rotation_2d(0)*after_rotation_2d(0) + after_rotation_2d(1)*after_rotation_2d(1)) + cam_est(8)*(after_rotation_2d(0)*after_rotation_2d(0) + after_rotation_2d(1)*after_rotation_2d(1))*(after_rotation_2d(0)*after_rotation_2d(0) + after_rotation_2d(1)*after_rotation_2d(1));
		Eigen::Matrix<double,2,1> result = cam_est(6) * radius * after_rotation_2d;
		_error = _measurement - result;
	}

	virtual void linearizeOplus()
	{
		const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> (vertex(0));
		const VertexPointBAL* point = static_cast<const VertexPointBAL*> (vertex(1));
		Eigen::Matrix<double,9,1> cam_est = cam->estimate();
		Eigen::Matrix<double,3,1> point_est = point->estimate();

		Eigen::Matrix<double,3,1> rotation_part(cam_est(3),cam_est(4),cam_est(5));
		Eigen::AngleAxisd rotation_vector(rotation_part.norm(), rotation_part/rotation_part.norm());
		Eigen::Matrix<double,3,1> translation_part(cam_est(0), cam_est(1), cam_est(2));
		Eigen::Matrix<double,3,1> after_rotation = rotation_vector.matrix()*point_est + translation_part;

		double X = after_rotation(0);
		double Y = after_rotation(1);
		double Z = after_rotation(2);

		// double X = point_est(0); // not sure at this point, which coordinate system?
		// double Y = point_est(1); // shoule be in camera coordinate system
		// double Z = point_est(2); // need to convert, current is world coordinate system


		double f = cam_est(6);
		double k1 = cam_est(7);
		double k2 = cam_est(8);




		Eigen::Matrix<double,2,6> j1 = Eigen::Matrix<double,2,6>::Zero();
		Eigen::Matrix<double,6,9> j2 = Eigen::Matrix<double,6,9>::Zero();
		 
		j1(0,0) = f/Z + 3*k1*f*pow(X,2.0)/pow(Z,3.0) + k1*f*pow(Y,2.0)/pow(Z,3.0) + 5*k2*f*pow(X,4.0)/pow(Z,5.0) + k2*f*pow(Y,4.0)/pow(Z,5.0) + 6*k2*f*pow(X,2.0)*pow(Y,2.0)/pow(Z,5.0);
		j1(1,0) = 2*k1*f*X*Y/pow(Z,3.0) + 4*k2*f*pow(X,3.0)*Y/pow(Z,5.0) + 4*k2*f*X*pow(Y,3.0)/pow(Z,5.0);
		j1(0,1) = j1(1,0);
		j1(1,1) = f/Z + k1*f*pow(X,2.0)/pow(Z,3.0) + 3*k1*f*pow(Y,2.0)/pow(Z,3.0) + k2*f*pow(X,4.0)/pow(Z,5.0) + 5*k2*f*pow(Y,4.0)/pow(Z,5.0) + 6*k2*f*pow(X,2.0)*pow(Y,2.0)/pow(Z,5.0);
		j1(0,2) = -f*X/(Z*Z) - 3*(k1*f*X*(X*X+Y*Y))/pow(Z,4.0) - 5*(k2*f*X*(pow(X,4.0)+pow(Y,4.0)+2*X*X*Y*Y))/pow(Z,6.0);
		j1(1,2) = -f*Y/(Z*Z) - 3*(k1*f*Y*(X*X+Y*Y))/pow(Z,4.0) - 5*(k2*f*Y*(pow(X,4.0)+pow(Y,4.0)+2*X*X*Y*Y))/pow(Z,6.0);
		j1(0,3) = X/Z + k1*X*(X*X+Y*Y)/(Z*Z*Z) + k2*X*(pow(X,4.0)+pow(Y,4.0)+2*X*X*Y*Y)/pow(Z,5.0); //wrt f
		j1(1,3) = Y/Z + k1*Y*(X*X+Y*Y)/(Z*Z*Z) + k2*Y*(pow(X,4.0)+pow(Y,4.0)+2*X*X*Y*Y)/pow(Z,5.0);
		j1(0,4) = f*X*(X*X+Y*Y)/(Z*Z*Z); // wrt k1
		j1(1,4) = f*Y*(X*X+Y*Y)/(Z*Z*Z);
		j1(0,5) = f*X*(X*X+Y*Y)*(X*X+Y*Y)/pow(Z,5.0);// wrt k2
		j1(1,5) = f*Y*(X*X+Y*Y)*(X*X+Y*Y)/pow(Z,5.0);

		j2(0,0) = 1;
		j2(1,1) = 1;
		j2(2,2) = 1;
		j2(0,4) = Z;
		j2(0,5) = -Y;
		j2(1,3) = -Z;
		j2(1,5) = X;
		j2(2,3) = Y;
		j2(2,4) = -X;
		j2(3,6) = 1;
		j2(4,7) = 1;
		j2(5,8) = 1;

		_jacobianOplusXi = j1*j2;

		Eigen::Matrix<double,2,3> j3 = j1.block<2,3>(0,0);

		
		_jacobianOplusXj = j3 * rotation_vector.matrix();
	}

};