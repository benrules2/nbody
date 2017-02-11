This project contains a C++ and Python gravity simulation.

These are tied to blog posts found:
http://www.cyber-omelette.com/2016/11/python-n-body-orbital-simulation.html
http://www.cyber-omelette.com/2017/02/RK4.html

The python program is intended as an introduction to the concepts. To access the python project, navigate to Python_Orbits. There is a visual studio python project in that directory, or the
raw source files can be found in the "Planet Smasher" directory.


The C++ program will be written for performance and additional integration techniques.

C++:

To access C++ application, navigate to the Cpp_Orbits directory.

A C++ N body simulation application

to compile, create a build directory and run cmake ($path_to_this_dir)

Then make should compile the software, or if you're using visual studio there will
be a project file in the build directory you can open and compile.

When you run this program it will output a .dat file for each body in the simluation.
To view your output, run:

python plot.py [.dat files]

This will plot all listed .dat files together on a 3d plot.


