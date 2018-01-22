#ifndef PROBLEM_H
#define PROBLEM_H


#include <vector>
#include <string>
#include "g2o/core/sparse_optimizer.h"
using namespace std;

class problem
{
public:
	problem(string file_name);
	~problem();

	


	// int num_cameras() const {return num_cameras_}
	// int num_points() const {return num_points_}
	// int num_observations() const {return num_observations_}
	// int num_parameters_



private:
	void construct();
	void solve();

	int num_cameras_, num_points_, num_observations_;
	int num_parameters_;

	int* point_index_;
	int* camera_index_; 
	double* observations_; 
	double* parameters_;

};

#endif