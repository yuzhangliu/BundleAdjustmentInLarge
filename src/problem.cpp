#include "problem.h"
#include "g2o_bal_class.h"
//using namespace std;

problem::problem(string file_name)
{
	ifstream input_file(file_name);
	
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
	}	
	else
	{
		cerr<<"Cannot open file"<<endl;
	}
}

problem::~problem()
{
	delete [] camera_index_;
	delete [] point_index_;
	delete [] observations_;
	delete [] parameters_;
}



void problem::construct()
{
	

}


void problem::solve()
{

}