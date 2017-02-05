A C++ N body simulation application

to compile, create a build directory and run cmake ($path_to_this_dir)

Then make should compile the software, or if you're using visual studio there will
be a project file in the build directory you can open and compile.

When you run this program it will output a .dat file for each body in the simluation.
To view your output, run:

python plot.py [.dat files]

This will plot all listed .dat files together on a 3d plot.