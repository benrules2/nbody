#include <math.h>

#include "datastructures.h"
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
			acceleration = acceleration + (external_body->location - target_body.location) * tmp;
		}
	}
	return acceleration;
}

void Orbit_integration::Euler::compute_velocity()
{
	for (int i = 0; i < m_bodies.size(); i++)
	{
		point acceleration = Orbit_integration::Euler::calculate_single_body_acceleration(i);
		m_bodies[i].velocity += acceleration * m_time_step;
	}
};

void Orbit_integration::Euler::update_location()
{
	for (auto target_body = m_bodies.begin(); target_body != m_bodies.end(); *target_body++)
	{
		target_body->location += target_body->velocity * m_time_step;
	}
}

void Orbit_integration::Euler::compute_gravity_step()
{
	compute_velocity();
	update_location();
}


point Orbit_integration::RK4::calculate_single_body_acceleration(int body_index)
{
	double G_const = 6.67408e-11; //m3 kg - 1 s - 2
	point acceleration{ 0, 0, 0 };
	point velocity_update{ 0, 0, 0 };
	point location_update{ 0, 0, 0 };
	body target_body = m_bodies[body_index];

	int index = 0;
	for (auto external_body = m_bodies.begin(); external_body != m_bodies.end(); *external_body++, index++)
	{
		if (index != body_index)
		{
			point k1{ 0, 0, 0 };
			point k2{ 0, 0, 0 };
			point k3{ 0, 0, 0 };
			point k4{ 0, 0, 0 };

			double r = (pow((target_body.location.x - external_body->location.x), 2) + 
						pow((target_body.location.y - external_body->location.y), 2) + 
						pow((target_body.location.z - external_body->location.z), 2));

			r = sqrt(r);

			auto tmp = G_const * external_body->mass / (r*r*r);

			//k1 - acceleration at current location
			k1 = (external_body->location - target_body.location) * tmp;

			//k2 - acceleration 0.5 timesteps in the future based on k1 acceleration value
			velocity_update = partial_step(target_body.velocity, k1, 0.5);
			location_update = partial_step(target_body.location, velocity_update, 0.5);
			k2 = (external_body->location - location_update) * tmp;

			//k3 acceleration 0.5 timesteps in the future using k2 acceleration
			velocity_update = partial_step(target_body.velocity, k2, 0.5);
			location_update = partial_step(target_body.location, velocity_update, 0.5);
			k3 = (external_body->location - location_update) * tmp;

			//k4 - location 1 timestep in the future using k3 acceleration
			velocity_update = partial_step(target_body.velocity, k3, 1);
			location_update = partial_step(target_body.location, velocity_update, 1);
			k4 = (external_body->location - location_update) * tmp;

			acceleration += (k1 + k2 * 2 + k3 * 2 + k4) / 6;
		}
	}
	return acceleration;
}

point Orbit_integration::RK4::partial_step(point &f, point &df, double scale)
{
	return point{
		f.x + df.x * m_time_step * scale,
		f.y + df.y * m_time_step * scale,
		f.z + df.z * m_time_step * scale
	};
};


void Orbit_integration::RK4::compute_velocity()
{
	for (int i = 0; i < m_bodies.size(); i++)
	{
		point acceleration = Orbit_integration::RK4::calculate_single_body_acceleration(i);
		m_bodies[i].velocity += acceleration * m_time_step;
	}
};

void Orbit_integration::RK4::update_location()
{
	for (auto target_body = m_bodies.begin(); target_body != m_bodies.end(); *target_body++)
	{
		target_body->location += target_body->velocity * m_time_step;
	}
}

void Orbit_integration::RK4::compute_gravity_step()
{
	compute_velocity();
	update_location();
}


