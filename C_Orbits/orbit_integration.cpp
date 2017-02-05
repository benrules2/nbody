#include <math.h>
#include <iostream>
#include <fstream>

#include "orbit_integration.h"


point Orbit_integration::Euler::calculate_single_body_acceleration(int body_index)
{
	double G_const = 6.67408e-11; //m3 kg - 1 s - 2
	point acceleration{ 0, 0, 0 };
	body target_body = m_bodies[body_index];

	int index = 0;

	for (auto external_body = m_bodies.begin(); external_body != m_bodies.end(); *external_body++, index++)
	{
		if (index != body_index)
		{
			double r = (pow((target_body.location.x - external_body->location.x), 2) + pow((target_body.location.y - external_body->location.y), 2) + pow((target_body.location.z - external_body->location.z), 2));
			r = sqrt(r);
			auto tmp = G_const * external_body->mass / (r*r*r);
			acceleration.x += tmp * (external_body->location.x - target_body.location.x);
			acceleration.y += tmp * (external_body->location.y - target_body.location.y);
			acceleration.z += tmp * (external_body->location.z - target_body.location.z);
		}
	}
	return acceleration;
}

void Orbit_integration::Euler::compute_velocity()
{
	int body_index = 0;
#pragma omp for
	for (int i = 0; i < m_bodies.size(); i++)
	{
		auto target_body = &m_bodies[i];
		point acceleration = Orbit_integration::Euler::calculate_single_body_acceleration(body_index);
#pragma omp atomic
		target_body->velocity.x += acceleration.x * m_time_step;
#pragma omp atomic
		target_body->velocity.y += acceleration.y * m_time_step;
#pragma omp atomic
		target_body->velocity.z += acceleration.z * m_time_step;
	}
};

void Orbit_integration::Euler::update_location()
{
	for (auto target_body = m_bodies.begin(); target_body != m_bodies.end(); *target_body++)
	{
		target_body->location.x += target_body->velocity.x * m_time_step;
		target_body->location.y += target_body->velocity.y * m_time_step;
		target_body->location.z += target_body->velocity.z * m_time_step;
	}
}

void Orbit_integration::Euler::compute_gravity_step()
{
	compute_velocity();
	update_location();
}

void Orbit_integration::Euler::record_state()
{
	if (!m_record_init)
	{
		for (auto body_iterator = m_bodies.begin(); body_iterator != m_bodies.end(); *body_iterator++)
		{
			std::ofstream f;
			f.open(body_iterator->name + ".dat");
			f << body_iterator->name << std::endl;
			f.close();
		}
		m_record_init = true;
	}

	for (auto body_iterator = m_bodies.begin(); body_iterator != m_bodies.end(); *body_iterator++)
	{
		std::ofstream f;
		f.open(body_iterator->name + ".dat", std::ofstream::out | std::ofstream::app);
		f << body_iterator->location.x << ","
			<< body_iterator->location.y << ","
			<< body_iterator->location.z << std::endl;
		f.close();
	}
}
