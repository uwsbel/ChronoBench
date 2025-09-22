import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# =============================================================================

class MyDriver (veh.ChDriver):
	def __init__(self, vehicle, delay):
		veh.ChDriver.__init__(self, vehicle)
		self.delay = delay
	def Synchronize(self, time):
		eff_time = time - self.delay
		if (eff_time < 0):
		    return

		if (eff_time > 0.2):
			self.SetThrottle(0.7)
		else:
			self.SetThrottle(3.5 * eff_time)

		if (eff_time < 2):
			self.SetSteering(0.0)
		else:
			self.SetSteering(0.6 * m.sin(2.0 * m.pi * (eff_time - 2) / 6))

		self.SetBraking(0.0)

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the various modules of the vehicle system

    # Create the vehicle, set parameters, and initialize
    car = veh.CarMMC()
    car.SetContactMethod(chrono.ChContactMethod_SMC)
    car.SetChassisFixed(False);
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    car.SetEngineType(veh.EngineModelType_SHAFTS);
    car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    car.SetTireType(veh.TireModelType_TMEASY)
    car.SetTireStepSize(1e-3)
    car.Initialize()

    car.SetChassisVisualizationType(veh.VisualizationType_NONE)
    car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(veh.VisualizationType_MESH)
    car.SetTireVisualizationType(veh.VisualizationType_MESH)

    car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('MMC')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    # Create the interactive driver system
    driver = MyDriver(car.GetVehicle(), 0.5)
    driver.Initialize()

    # Enable real-time driving
    veh.EnableRealtime(True)

    # Simulation loop
    while vis.Run() :
        time = car.GetSystem().GetChTime()

        # End simulation
        if (time >= 4):
            break

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        car.Synchronize(time, driver_inputs, vis)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        car.Advance(step_size)
        vis.Advance(step_size)

    return 0
  

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step size
step_size = 2e-3


main()