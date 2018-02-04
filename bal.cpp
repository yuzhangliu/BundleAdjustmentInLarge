#include <iostream>
//#include "utility.h"
#include "problem.h"

using namespace std;

#define PII 3.14159
int main(int argc, char** argv)

{
	// test utility function
	// double a[3] = {0,0,PII/4.0};
	// double b[3] = {5.0,3.0,4.0};
	// double c[3];
	// CameraToWorld(a,b,c);
	// for (int i = 0; i < 3; i++)
	// {
	// 	cout<<c[i]<<" ";
	// }
	// cout<<endl;


	// test problem class
	if (argc != 4)
	{
		cout<<"Usage: ./bal path_to_data_file path_to_initial_ply_file path_to_final_ply_file"<<endl;
		return 0;
	}

	string data_file_name(argv[1]);
	string initial_ply_file_name(argv[2]);
	string final_ply_file_name(argv[3]);

	problem prob(data_file_name, initial_ply_file_name, final_ply_file_name);
	prob.write_initial();
	prob.solve();
	prob.write_final();

	return 0;
}