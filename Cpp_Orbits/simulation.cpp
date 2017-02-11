#include "orbit_integration.h"

template <typename Integrator>
void run_simulation(Integrator integrator, int iterations, int report_frequency)
{
	for (auto i = 0; i < iterations; i++)
	{
		if (i % report_frequency == 0)
			record_state(integrator.get_bodies());
		integrator.compute_gravity_step();
	}
	output_states(integrator.get_bodies());
}

int main(int argc, char *argv[])
{
	std::vector<body> bodies;
	
	bodies.push_back(solar_system::sun);
	//bodies.push_back(solar_system::mercury);
	bodies.push_back(body{ { 0, 1.0e10, 0 }, 1e23,{ 3e4, 0, 0 }, "sat" });
	//bodies.push_back(solar_system::venus);
	//bodies.push_back(solar_system::earth);
	//bodies.push_back(solar_system::mars);
	//bodies.push_back(solar_system::saturn);
	//bodies.push_back(solar_system::jupiter);
	//bodies.push_back(solar_system::uranus);
	//bodies.push_back(solar_system::neptune);
	//bodies.push_back(solar_system::pluto);
	//
	Orbit_integration::Euler orbit(bodies, 1);
	
	run_simulation(orbit, (int)1e7, 100);

	printf("Done\n");

}