import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.typedefs as td

# Initialize the PyChrono environment
chrono.SetChronoDataPath('../../data/')

# Create the simulation system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the Irrlicht visualization system
vis = chronoirr.ChIrrApp(system, 'Gator Vehicle Simulation', chrono.ChVectorD(0.2, 0.2, 0.9))

# Create the terrain
terrain = veh.ChTerrain(0.1)
terrain.SetTexture(chrono.GetChronoDataFile('terrain/textures/tile1.jpg'), True)
terrain.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
terrain.Initialize()

# Add the terrain to the system
terrain_body = terrain.GetGroundBody()
system.Add(terrain_body)
terrain_body.SetPos(chrono.ChVectorD(0, 0, 0))

# Create the vehicle
vehicle = veh.ChVehicle(veh.ChVehicleType::GATOR)
vehicle.SetChassisBody(chrono.ChBodyEasyBox(1.5, 1.5, 0.5, 1000, True, True, chrono.ChMaterialSurfaceNSC()))
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1.5))
vehicle.SetInitOrientation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1)))
vehicle.SetTireType(veh.ChTireHandler::TMEASY)
vehicle.SetTireVisualizationType(veh.ChTireHandler::MESH)

# Add the vehicle to the system
vehicle_body = vehicle.GetChassisBody()
system.Add(vehicle_body)

# Set up the interactive driver
driver = veh.ChDriver()
driver.Initialize(vehicle, system)
driver.SetSteeringDelta(0.0)  # Initial steering angle
driver.SetThrottle(0.0)       # Initial throttle
driver.SetBraking(0.0)        # Initial braking

# Set up the simulation parameters
system.Set_SolverType(chrono.ChSolver::Type::BARZILAIBORWEIN)
system.Set_TimestepperType(chrono.ChTimestepper::Type::HHT)
system.Set_Timestep(1.0 / 50)  # 50 frames per second

# Run the simulation
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'), 5, 5)
vis.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 1, 1, 50, 50, 0.2)
vis.SetSymbolScale(0.02)
vis.SetContactDrawing(True)
vis.SetContactDrawingSize(0.01)
vis.SetContactDrawingColors(chrono.ChColor(1, 0, 0), chrono.ChColor(0, 1, 0))

vis.AddVisualSystem(chrono.ChMaterialSurfaceNSC())
vis.SetWindowSize(1280, 720)
vis.SetWindowPosition(0, 0)
vis.Initialize()
vis.AddVisualSystem(chrono.ChMaterialSurfaceNSC())

# Simulation loop
while vis.Run():
    driver.SynchronizeForces(vehicle)
    system.DoStepDynamics(1.0 / 50)
    vis.Render()

# Clean up
vis.Close()