#include <iostream>

#include "problem.h"

using namespace std;

int main(int argc, char** argv)

{
	string file_name = "../data/problem-21-11315-pre.txt";
	problem prob(file_name);
	prob.construct();
	prob.solve();
	prob.write();
	// std::cout<<prob.param<<std::endl;
	// prob.setparam(20);
	// std::cout<<prob.param<<std::endl;

	return 0;
}