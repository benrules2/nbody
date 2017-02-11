#pragma once

#include <iostream>
#include <fstream>

#include "datastructures.h"


static void record_state(std::vector<body>& bodies)
{
	for (auto body_iterator = bodies.begin(); body_iterator != bodies.end(); *body_iterator++)
	{
		body_iterator->locations.push_back(body_iterator->location);
	}
}

static void output_states(const std::vector<body>& body_locations)
{

	for (auto body_iterator = body_locations.begin(); body_iterator != body_locations.end(); *body_iterator++)
	{
		std::ofstream f;
		f.open(body_iterator->name + ".dat");
		f << body_iterator->name << std::endl;
		f.close();
	}

	for (auto body_iterator = body_locations.begin(); body_iterator != body_locations.end(); *body_iterator++)
	{
		std::ofstream f;
		f.open(body_iterator->name + ".dat", std::ofstream::out | std::ofstream::app);
		for (auto location = body_iterator->locations.begin(); location < body_iterator->locations.end(); *location++)
		{
			f << location->x << ","
				<< location->y << ","
				<< location->z << std::endl;
		}
		f.close();
	}
}


// location(x, y, z), mass (kg), velocity (m/s), name
namespace solar_system
{
	static body sun{ { 0, 0, 0 }, 2e30,{ 0,0,0 }, "sun" };
	static body mercury{ { 0, 5.0e10, 0 }, 3.285e23,{ 47000, 0, 0 }, "mercury" };
	static body venus{ { 0, 1.1e11, 0 }, 4.8e24,{ 35000, 0, 0 }, "venus" };
	static body earth{ { 0, 1.5e11, 0 }, 6e24,{ 30000,0,0 }, "earth" };
	static body mars{ { 0, 2.2e11, 0 }, 2.4e24,{ 24000,0,0 }, "mars" };
	static body jupiter{ { 7.7e11,0,0 }, 1e28,{ 0,13000,0 }, "jupiter" };
	static body saturn{ { 0, 1.4e12, 0 }, 5.7e26,{ 9000,0,0 }, "saturn" };
	static body uranus = { { 0, 2.8e12, 0 },  8.7e25,{ 6835, 0, 0 }, "uranus" };
	static body neptune = { { 0, 4.5e12, 0 }, 1e26,{ 5477,0,0 }, "neptune" };
	static body pluto = { { 0, 7.3e12, 0 }, 1.3e22,{ 4748,0,0 }, "pluto" };
}


namespace Orbit_integration
{
	class Integrator
	{
	public:
		virtual void compute_gravity_step() = 0;
		virtual std::vector<body>& get_bodies() = 0;
	};

	class Euler : virtual Integrator
	{
	public:
		Euler(std::vector<body> bodies, double time_step = 1) :
			m_bodies(bodies),
			m_time_step(time_step) 
		{};

		std::vector<body>& get_bodies() { return m_bodies; };
		void compute_gravity_step();

	private:
		point calculate_single_body_acceleration(int);
		void compute_velocity();
		void update_location();

	protected:
		std::vector<body> m_bodies;
		double m_time_step;
	};

	class RK4 : virtual public Integrator
	{
	public:
		RK4(std::vector<body> bodies, double time_step = 1) :
			m_bodies(bodies),
			m_time_step(time_step)
		{};

		std::vector<body>& get_bodies() { return m_bodies; };
		void compute_gravity_step();

	private:
		point calculate_single_body_acceleration(int);
		point partial_step(point &, point &, double);
		void compute_velocity();
		void update_location();

	protected:
		std::vector<body> m_bodies;
		double m_time_step;
	};
}
