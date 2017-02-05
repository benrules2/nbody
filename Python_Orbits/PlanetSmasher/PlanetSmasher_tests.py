import PlanetSmasher as planets

import unittest

class Test_PlanetSmasher_tests(unittest.TestCase):
    def test_Acceleration_Calc(self):
        locations = [planets.point(0, 0,0), planets.point(149.6e9, 0,0)]
        masses = [2e30, 6000]
        accel_sun = planets.calculate_single_body_acceleration(locations, masses, 0, locations[0])
        accel_earth = planets.calculate_single_body_acceleration(locations, masses, 1, locations[1])
        self.assertTrue( abs(accel_earth.x) > abs(accel_sun.x))


    def test_Similar_Body_accel(self):
        locations = [planets.point(10, 10, 10), planets.point(-10, 10, 10)]
        masses = [1000, 1000]
        accel_0 = planets.calculate_single_body_acceleration(locations, masses, 0, locations[0])
        accel_1 = planets.calculate_single_body_acceleration(locations, masses, 1, locations[1])
        self.assertTrue( accel_1.x == -1 * accel_0.x)

    def test_simulation(self):
        locations =  [planets.point(0,0,0), planets.point(0,149.6e9,100), planets.point(221e9,0,0)]        
        velocities = [planets.point(0,0,0), planets.point(-30000,0,0), planets.point(0,24000,0)]        
        masses = [2e30, 6e24, 2.4e24]                  

        movements = planets.run_simulation(locations, velocities, masses, number_of_steps = 100000, time_step = 100, report_freq = 1000)
                
        planets.plot_output(movements)

if __name__ == '__main__':
    unittest.main()
