import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import os

# Initialize Chrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Create CityBus vehicle
veh = vehicle.CityBus(system)
veh.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
veh.SetInitFwdDirection(chrono.ChVector3d(1, 0, 0))
veh.Initialize()

# Set visualization types
veh.SetChassisVisualizationType(vehicle.VisualizationType.MESH)
veh.SetSuspensionVisualizationType(vehicle.VisualizationType.PRIMITIVES)
veh.SetSteeringVisualizationType(vehicle.VisualizationType.PRIMITIVES)
veh.SetWheelVisualizationType(vehicle.VisualizationType.PRIMITIVES)

# Create rigid terrain with texture
terrain = vehicle.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetTexture(veh.GetChronoDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()

# Set up Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -10, 3), chrono.ChVector3d(0, 0, 1.2))
vis.AddTypicalLights()

# Create interactive driver
driver = vehicle.InteractiveDriverIRR(vis, veh)
driver.Initialize()

# Simulation loop settings
time_step = 1 / 50

# Main simulation loop
while vis.Run():
    # Update camera to follow vehicle
    chassis_pos = veh.GetChassisBody().GetPos()
    vis.SetCamera(chrono.ChVector3d(chassis_pos.x - 8, chassis_pos.y, chassis_pos.z + 3), chassis_pos)

    vis.BeginScene()
    vis.Render()

    # Get and apply driver inputs
    driver_inputs = driver.GetInputs()
    driver.Update()

    system.DoStepDynamics(time_step)
    vis.EndScene()