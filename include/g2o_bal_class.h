#include <Eigen/Core>

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

//typedef Eigen::Matrix<double, 9, 1> Vector9d;

class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexCameraBAL(){}
	virtual bool read ( std::istream& in){}
	virtual bool write ( std::ostream& out) const {}
	virtual void setToOriginImpl(){}

	virtual void oplusImpl (const double* update)
	{
		Eigen::Matrix<double, 6, 1>::ConstMapType v1(update, 6); 
		Eigen::Matrix<double, 3, 1>::ConstMapType v2(update+6, 3);
		for (int i = 0; i < 3; i++)
		{
			_estimate(i+6) += v2(i);
		}
		Eigen::Matrix<double,6,1> after = (Sophus::SE3::exp(v1) *).log(); 
		Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
		_estimate += v;
	}
};


class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	VertexPointBAL(){}

	virtual bool read ( std::istream& in){}
	virtual bool write ( std::ostream& out) const {}
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

	virtual bool read( std::istream& in){}
	virtual bool write(std::ostream& out){}

	virtual void computeError()
	{

	}

	virtual void linearizeOplus()
	{

	}

};