import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math as m

# Custom driver class
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        veh.ChDriver.__init__(self, vehicle)
        self.delay = delay

    def Synchronize(self, time):
        eff_time = time - self.delay
        if eff_time < 0:
            return

        if eff_time > 0.2:
            self.SetThrottle(0.7)
        else:
            self.SetThrottle(3.5 * eff_time)

        if eff_time < 2:
            self.SetSteering(0.0)
        else:
            self.SetSteering(0.6 * m.sin(2.0 * m.pi * (eff_time - 2) / 6))

        self.SetBraking(0.0)

# Rest of the script remains the same

# Create the (custom) driver
driver = MyDriver(vehicle.GetVehicle(), 0.5)
driver.Initialize()

# ...

# Simulation end condition
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # End simulation
    if time >= 4:
        break

    # ...