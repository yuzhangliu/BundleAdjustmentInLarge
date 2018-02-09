#ifndef PROBLEM_H
#define PROBLEM_H


#include <vector>
#include <string>

#include "g2o_bal_class.h"
#include "utility.h"

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"





using namespace std;

class problem
{
public:
	problem(string data_file_name, string initial_ply_file_name, string final_ply_file_name);
	~problem();
	void write_initial();
	void write_final();

	void solve();



	const double* parameters(){return parameters_;}




	// int num_cameras() const {return num_cameras_}
	// int num_points() const {return num_points_}
	// int num_observations() const {return num_observations_}
	// int num_parameters_



private:
	


	string data_file_name_;
	string initial_ply_file_name_;
	string final_ply_file_name_;
	


	int num_cameras_, num_points_, num_observations_;
	int num_parameters_;

	int* point_index_;
	int* camera_index_; 
	double* observations_; 
	double* parameters_;

};

#endif