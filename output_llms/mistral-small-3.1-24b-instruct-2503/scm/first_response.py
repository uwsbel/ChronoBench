import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.terrain as ter
import pychrono.solver as solver

# Initialize the Chrono::Engine
chrono.SetChronoDataPath('../../data/')

# Create the simulation system and add a gravity field
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the HMMWV vehicle
vehicle = veh.HMMWV()
vehicle.InitSystem(system, veh.ChVehicleModelType.VEHICLE_3D)
vehicle.SetChassisVisualizationType(veh.ChVehicleVisualizationType.MESH)

# Set the initial position and orientation of the HMMWV
vehicle.SetInitPosition(chrono.ChVectorD(0, 1, 0))
vehicle.SetInitOrientation(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))

# Set up the terrain
terrain = ter.ChTerrain()
terrain.Initialize(system, "terrain/scmtest")
terrain.SetContactMaterialProperties(0.3, 0.01, 0.01, 0.01, 0.0, 0.0, 0.0)

# Enable moving patch feature
terrain.EnableMovingPatch(vehicle.GetChassisBody(), 3, 3, 0.2, 0.2)

# Enable false color plotting for sinkage visualization
terrain.EnableSinkagePlotting()

# Set up the interactive driver
driver = veh.ChDriver()
driver.Initialize(vehicle.GetVehicle(), system)
driver.SetSteeringController(veh.ChDriver::STEERING_CONTROLLER_SIMPLE)
driver.SetThrottleController(veh.ChDriver::THROTTLE_CONTROLLER_SIMPLE)
driver.SetBrakingController(veh.ChDriver::BRAKING_CONTROLLER_SIMPLE)

# Set up the visualization
visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('HMMWV on SCM Terrain')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLights()
visualization.AddLogo()
visualization.AddLightWithShadow(chrono.ChVectorD(1.0, -1.0, 1.0), chrono.ChVectorD(0, 0, 0), 1, 1.5, 30)

# Set the simulation step size and real-time factor
system.SetTimestepperType(chrono.ChTimestepperType.HYBRID)
system.SetSolverType(solver.ChSolverType.BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-10)
system.SetStep(0.01)

# Run the simulation in real-time at 50 frames per second
run_time = 10  # seconds
current_time = 0
while current_time < run_time:
    visualization.Render()
    system.DoStepDynamics(0.01)
    current_time += 0.01

# Clean up
visualization.Close()