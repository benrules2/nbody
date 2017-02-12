import math
import random
import matplotlib.pyplot as plot
from mpl_toolkits.mplot3d import Axes3D

class point:
    def __init__(self, x,y,z):
        self.x = x
        self.y = y
        self.z = z

class body:
    def __init__(self, location, mass, velocity, name = ""):
        self.location = location
        self.mass = mass
        self.velocity = velocity
        self.name = name

def partial_step(point1, point2, time_step):
    ret = point(0,0,0)
    ret.x = point1.x + point2.x * time_step
    ret.y = point1.y + point2.y * time_step
    ret.z = point1.z + point2.z * time_step
    return ret

class Euler_integrator:
    def __init__(self, time_step, bodies):
        self.time_step = time_step
        self.bodies = bodies

    def calculate_single_body_acceleration(self, body_index):
        G_const = 6.67408e-11 #m3 kg-1 s-2
        acceleration = point(0,0,0)
        target_body = self.bodies[body_index]
        for index, external_body in enumerate(bodies):
            if index != body_index:
                r = (target_body.location.x - external_body.location.x)**2 + (target_body.location.y - external_body.location.y)**2 + (target_body.location.z - external_body.location.z)**2
                r = math.sqrt(r)
                tmp = G_const * external_body.mass / r**3
                acceleration.x += tmp * (external_body.location.x - target_body.location.x)
                acceleration.y += tmp * (external_body.location.y - target_body.location.y)
                acceleration.z += tmp * (external_body.location.z - target_body.location.z)
    
        return acceleration
    
    def update_location(self):
        for target_body in self.bodies:
            target_body.location.x += target_body.velocity.x * self.time_step
            target_body.location.y += target_body.velocity.y * self.time_step
            target_body.location.z += target_body.velocity.z * self.time_step
           
    def compute_velocity(self):
        for body_index, target_body in enumerate(self.bodies):
            acceleration = self.calculate_single_body_acceleration(body_index)
            target_body.velocity.x += acceleration.x * self.time_step
            target_body.velocity.y += acceleration.y * self.time_step
            target_body.velocity.z += acceleration.z * self.time_step 
    
    
    def update_location(self):
        for target_body in self.bodies:
            target_body.location.x += target_body.velocity.x * self.time_step
            target_body.location.y += target_body.velocity.y * self.time_step
            target_body.location.z += target_body.velocity.z * self.time_step

    def compute_gravity_step(self):
        self.compute_velocity()
        self.update_location()


  

class RK4_integrator:
    def __init__(self, time_step, bodies):
        self.time_step = time_step
        self.bodies = bodies

    def calculate_single_body_acceleration(self, body_index):
        G_const = 6.67408e-11 #m3 kg-1 s-2
        acceleration = point(0,0,0)
        target_body = self.bodies[body_index]

        k1 = point (0,0,0)
        k2 = point (0,0,0)
        k3 = point (0,0,0)
        k4 = point (0,0,0)
        tmp_loc = point (0,0,0)
        tmp_vel = point (0,0,0)

        for index, external_body in enumerate(self.bodies):
            if index != body_index:
                r = (target_body.location.x - external_body.location.x)**2 + (target_body.location.y - external_body.location.y)**2 + (target_body.location.z - external_body.location.z)**2
                r = math.sqrt(r)
                tmp = G_const * external_body.mass / r**3

                #k1 - regular Euler acceleration
                k1.x = tmp * (external_body.location.x - target_body.location.x)
                k1.y = tmp * (external_body.location.y - target_body.location.y)
                k1.z = tmp * (external_body.location.z - target_body.location.z)

                #k2 - acceleration 0.5 timesteps in the future based on k1 acceleration value
                tmp_vel = partial_step(target_body.velocity, k1, 0.5)
                tmp_loc = partial_step(target_body.location, tmp_vel, 0.5 * self.time_step)
                k2.x = (external_body.location.x - tmp_loc.x) * tmp
                k2.y = (external_body.location.y - tmp_loc.y) * tmp
                k2.z = (external_body.location.z - tmp_loc.z) * tmp

                #k3 acceleration 0.5 timesteps in the future using k2 acceleration
                tmp_vel = partial_step(target_body.velocity, k2, 0.5)
                tmp_loc = partial_step(target_body.location, tmp_vel, 0.5 * self.time_step)
                k3.x = (external_body.location.x - tmp_loc.x) * tmp
                k3.y = (external_body.location.y - tmp_loc.y) * tmp
                k3.z = (external_body.location.z - tmp_loc.z) * tmp

                #k4 - location 1 timestep in the future using k3 acceleration
                tmp_vel = partial_step(target_body.velocity, k3, 1)
                tmp_loc = partial_step(target_body.location, tmp_vel, self.time_step)
                k4.x = (external_body.location.x - tmp_loc.x) * tmp;
                k4.y = (external_body.location.y - tmp_loc.y) * tmp;
                k4.z = (external_body.location.z - tmp_loc.z) * tmp;

                acceleration.x += (k1.x + k2.x * 2 + k3.x * 2 + k4.x) / 6;
                acceleration.y += (k1.y + k2.y * 2 + k3.y * 2 + k4.y) / 6;
                acceleration.z += (k1.z + k2.z * 2 + k3.z * 2 + k4.z) / 6;

        return acceleration
    
    def update_location(self):
        for target_body in self.bodies:
            target_body.location.x += target_body.velocity.x * self.time_step
            target_body.location.y += target_body.velocity.y * self.time_step
            target_body.location.z += target_body.velocity.z * self.time_step
           
    def compute_velocity(self):
        for body_index, target_body in enumerate(self.bodies):
            acceleration = self.calculate_single_body_acceleration(body_index)
            target_body.velocity.x += acceleration.x * self.time_step
            target_body.velocity.y += acceleration.y * self.time_step
            target_body.velocity.z += acceleration.z * self.time_step 
    
    
    def update_location(self):
        for target_body in self.bodies:
            target_body.location.x += target_body.velocity.x * self.time_step
            target_body.location.y += target_body.velocity.y * self.time_step
            target_body.location.z += target_body.velocity.z * self.time_step

    def compute_gravity_step(self):
        self.compute_velocity()
        self.update_location()
    



def plot_output(bodies, outfile = None):
    fig = plot.figure()
    colours = ['r','b','g','y','m','c']
    ax = fig.add_subplot(1,1,1, projection='3d')
    max_range = 0
    for current_body in bodies: 
        max_dim = max(max(current_body["x"]),max(current_body["y"]),max(current_body["z"]))
        if max_dim > max_range:
            max_range = max_dim
        ax.plot(current_body["x"], current_body["y"], current_body["z"], c = random.choice(colours), label = current_body["name"])        
    
    ax.set_xlim([-max_range,max_range])    
    ax.set_ylim([-max_range,max_range])
    ax.set_zlim([-max_range,max_range])
    ax.legend()        

    if outfile:
        plot.savefig(outfile)
    else:
        plot.show()

def run_simulation(integrator, names = None, number_of_steps = 10000, report_freq = 100):

    #create output container for each body
    body_locations_hist = []
    for current_body in bodies:
        body_locations_hist.append({"x":[], "y":[], "z":[], "name":current_body.name})
        
    for i in range(0, int(number_of_steps)):
        if i % report_freq == 0:
            for index, body_location in enumerate(body_locations_hist):
                body_location["x"].append(bodies[index].location.x)
                body_location["y"].append(bodies[index].location.y)           
                body_location["z"].append(bodies[index].location.z)       
        integrator.compute_gravity_step()            

    return body_locations_hist        
            
#planet data (location (m), mass (kg), velocity (m/s)
sun = {"location":point(0,0,0), "mass":2e30, "velocity":point(0,0,0)}
mercury = {"location":point(0,5.0e10,0), "mass":3.285e23, "velocity":point(47000,0,0)}
venus = {"location":point(0,1.1e11,0), "mass":4.8e24, "velocity":point(35000,0,0)}
earth = {"location":point(0,1.5e11,1e10), "mass":6e24, "velocity":point(30000,0,0)}
mars = {"location":point(0,2.2e11,0), "mass":6.4e23, "velocity":point(24000,0,0)}
jupiter = {"location":point(0,7.7e11,0), "mass":1e28, "velocity":point(13000,0,0)}
saturn = {"location":point(0,1.4e12,0), "mass":5.7e26, "velocity":point(9000,0,0)}
uranus = {"location":point(0,2.8e12,0), "mass":8.7e25, "velocity":point(6835,0,0)}
neptune = {"location":point(0,4.5e12,0), "mass":1e26, "velocity":point(5477,0,0)}
pluto = {"location":point(0,3.7e12,0), "mass":1.3e22, "velocity":point(4748,0,0)}

sat =  { "location": point(0, 1.0e10, 0), "mass":1e23, "velocity":point(3e4, 0, 0)}

if __name__ == "__main__":

    #build list of planets in the simulation, or create your own
    bodies = [
        body( location = sun["location"], mass = sun["mass"], velocity = sun["velocity"], name = "sun"),
        body( location = mercury["location"], mass = mercury["mass"], velocity = mercury["velocity"], name = "sun"),
        body( location = venus["location"], mass = venus["mass"], velocity = venus["velocity"], name = "sun"),
        body( location = earth["location"], mass = earth["mass"], velocity = earth["velocity"], name = "earth"),
        body( location = mars["location"], mass = mars["mass"], velocity = mars["velocity"], name = "mars"),
        body( location = saturn["location"], mass = saturn["mass"], velocity = saturn["velocity"], name = "saturn"),
        body( location = jupiter["location"], mass = jupiter["mass"], velocity = jupiter["velocity"], name = "jupiter"),
        body( location = uranus["location"], mass = uranus["mass"], velocity = uranus["velocity"], name = "uranus"),
        body( location = neptune["location"], mass = neptune["mass"], velocity = neptune["velocity"], name = "neptune"),
        body( location = pluto["location"], mass = pluto["mass"], velocity = pluto["velocity"], name = "pluto")
        ]
    
    integrator = RK4_integrator(time_step = 10, bodies = bodies)
    motions = run_simulation(integrator, number_of_steps = 1e5, report_freq = 1e4)
    plot_output(motions) #, "mercury_euler.png")
