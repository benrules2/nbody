#include "orbit_integration.h"

template <typename Integrator>
void run_simulation(Integrator integrator, int iterations, int report_frequency)
{
	for (auto i = 0; i < iterations; i++)
	{
		if (i % report_frequency == 0)
			integrator.record_state();
		integrator.compute_gravity_step();
	}
}

int main(int argc, char *argv[])
{
	std::vector<body> bodies;

	body sun{ { 0, 0, 0 }, 2e30, { 0,0,0 }, "sun" };
	bodies.push_back(sun);

	body earth{ {0, 1.5e11, 1e10 }, 6e24, { 30000,0,0 }, "earth"};
	bodies.push_back(earth);
	
	body jupiter{ { 0,7.7e11,0}, 1e28, { 13000,0,0 }, "jupiter" };
	bodies.push_back(jupiter);

	Orbit_integration::Euler euler_orbit(bodies, 100);
	
	run_simulation(euler_orbit, 10000000, 100000);

	printf("Done\n");

}