#include "problem.h"

//using namespace std;

problem::problem(string data_file_name, string initial_ply_file_name, string final_ply_file_name)
{
	// set string members
	data_file_name_ = data_file_name;
	initial_ply_file_name_ = initial_ply_file_name;
	final_ply_file_name_ = final_ply_file_name;

	// read data file
	ifstream input_file(data_file_name_);	
	if (input_file.is_open())
	{
		string line;
		
		// get the first line, number of points, cameras and observations
		if (getline(input_file, line))
		{
			istringstream in(line);
			in >> num_cameras_ >> num_points_ >> num_observations_;
		}

		// get the second part, observations
		camera_index_ = new int[num_observations_];
		point_index_ = new int[num_observations_];
		observations_ = new double[num_observations_*2];
		for (int i = 0 ; i < num_observations_; i++)
		{
			if (getline(input_file, line))
			{
				istringstream in(line);
				in >> camera_index_[i] >> point_index_[i] >> observations_[i*2] >> observations_[i*2+1];
			}
		}
		
		// get the third part, parameters
		num_parameters_ = num_cameras_ * 9 + num_points_ * 3;
		parameters_ = new double[num_parameters_];
		// camera parameters, r(3),t(3),f,k1,k2 
		for (int i = 0; i < num_cameras_; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				if (getline(input_file, line))
				{
					istringstream in(line);
					in >> parameters_[i*9+j];
				}
			}
		}
		// point parameters, t(3)
		for (int i = 0; i < num_points_; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if(getline(input_file, line))
				{
					istringstream in(line);
					in >> parameters_[num_cameras_*9 + i*3 +j];
				}
			}
		}

		// close file
		input_file.close();
		cout<<"bal data file loaded..."<<endl;
		cout<<"bal problem has "<<num_cameras_ <<" cameras and " <<num_points_ <<" points."<<endl;
		cout<<"Forming "<<num_observations_<<" observations."<<endl;
	}	
	else
	{
		cerr<<"Cannot open data file"<<endl;
	}
}

problem::~problem()
{
	if (camera_index_ != nullptr) delete [] camera_index_;
	if (point_index_ != nullptr) delete [] point_index_;
	if (observations_ != nullptr) delete [] observations_;
	if (parameters_ != nullptr) delete [] parameters_;
}


void problem::write_initial()
{
	ofstream initial_file(initial_ply_file_name_);

	if (initial_file.is_open())
	{
		cout<<"Writing the data into initial ply file..."<<endl;
		// write the header
		initial_file << "ply" << '\n'
		<<"format ascii 1.0" << '\n'
		<<"element vertex " << num_cameras_ + num_points_ << '\n'
		<<"property float x" <<'\n'
		<<"property float y" <<'\n'
		<<"property float z" <<'\n'
		<<"property uchar red" << '\n'
		<<"property uchar green" << '\n'
		<<"property uchar blue" << '\n'
		<<"end_header" << std::endl;

		// write camera
		for (int i = 0; i < num_cameras_; i++)
		{
			double* camera_tcw = new double[3];
			CameraToWorld(parameters_+i*9, parameters_+i*9+3, camera_tcw);
			initial_file << camera_tcw[0] <<' '<<camera_tcw[1] <<' '<<camera_tcw[2]<<' '<<"0 255 0"<<endl;
			delete camera_tcw;
			// initial_file << parameters_[i*9+3] << ' ' << parameters_[i*9+4] << ' ' << parameters_[i*9+5] 
			// << ' '<<"0 255 0" << endl;
			
		}

		// write points
		for (int i = 0; i < num_points_; i++)
		{
			initial_file << parameters_[num_cameras_*9 + i*3] << ' ' << parameters_[num_cameras_*9 + i*3+1] << ' '
			<< parameters_[num_cameras_*9 + i*3+2] << ' '<<"255 255 255" << endl;
		}
		initial_file.close();
		cout<<"Initial ply file written!"<<endl;
	}
	else
	{
		cout<<"Unable to open initial file"<<endl;
	}
	return;
}

void problem::write_final()
{
	ofstream final_file(final_ply_file_name_);

	if (final_file.is_open())
	{
		cout<<"Writing the data into final ply file..."<<endl;
		// write the header
		final_file << "ply" << '\n'
		<<"format ascii 1.0" << '\n'
		<<"element vertex " << num_cameras_ + num_points_ << '\n'
		<<"property float x" <<'\n'
		<<"property float y" <<'\n'
		<<"property float z" <<'\n'
		<<"property uchar red" << '\n'
		<<"property uchar green" << '\n'
		<<"property uchar blue" << '\n'
		<<"end_header" << std::endl;

		// write camera
		for (int i = 0; i < num_cameras_; i++)
		{
			double* camera_tcw = new double[3];
			CameraToWorld(parameters_+i*9, parameters_+i*9+3, camera_tcw);
			final_file << camera_tcw[0] <<' '<<camera_tcw[1] <<' '<<camera_tcw[2]<<' '<<"0 255 0"<<endl;
			delete camera_tcw;
			// initial_file << parameters_[i*9+3] << ' ' << parameters_[i*9+4] << ' ' << parameters_[i*9+5] 
			// << ' '<<"0 255 0" << endl;
			
		}

		// write points
		for (int i = 0; i < num_points_; i++)
		{
			final_file << parameters_[num_cameras_*9 + i*3] << ' ' << parameters_[num_cameras_*9 + i*3+1] << ' '
			<< parameters_[num_cameras_*9 + i*3+2] << ' '<<"255 255 255" << endl;
		}
		final_file.close();
		cout<<"Final ply file written!"<<endl;
	}
	else
	{
		cout<<"Unable to open final file"<<endl;
	}
	return;
}


void problem::solve()
{
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>> balBlockSolver;
	g2o::SparseOptimizer optimizer;

	balBlockSolver* solver_ptr;
	g2o::LinearSolver<balBlockSolver::PoseMatrixType>* linearSolver;
	linearSolver = new g2o::LinearSolverCholmod<balBlockSolver::PoseMatrixType>();

	dynamic_cast<g2o::LinearSolverCholmod<balBlockSolver::PoseMatrixType>*>(linearSolver)
		->setBlockOrdering(true);
	solver_ptr = new balBlockSolver(linearSolver);

	g2o::OptimizationAlgorithmWithHessian* solver;
	solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	optimizer.setAlgorithm(solver);


	// add vertex and edge





	// start optimization
	cout<<"Optimization starts..."<<endl;
	optimizer.initializeOptimization();
	optimizer.setVerbose(true);
	optimizer.optimize(100);
	cout<<"Optimization completed!"<<endl;





	delete linearSolver;
	delete solver_ptr;
	delete solver_ptr;


	

}
