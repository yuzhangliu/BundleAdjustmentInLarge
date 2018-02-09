# BundleAdjustmentInLarge
This repo works on bundle adjustment with dataset coming from http://grail.cs.washington.edu/projects/bal/
# Compile
After git clone this repo,
cd BundleAdjustmentInLarge
mkdir build
cd build
cmake ..
make 
# Run
To run this bundle adjustment,
./bal path_to_data_file path_to_initial_ply path_to_final_ply
Eg.
./bal ../data/problem-16-22106-pre.txt ../initial.ply ../final.ply
