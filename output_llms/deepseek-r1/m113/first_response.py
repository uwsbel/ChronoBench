import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Initialize Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.Initialize(patch_mat)

# Create M113 vehicle
vehicle = veh.M113(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.Initialize()

# Set visualization modes
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelAssemblyVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)

# Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('M113 Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, -5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, -5), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 35, 512)

# Create interactive driver
driver = veh.InteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
vehicle.GetVehicle().EnableRealtime(True)

# Simulation loop parameters
step_size = 0.005
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    # Update visualization and inputs
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    vehicle.GetVehicle().Synchronize(0, driver_inputs)
    terrain.Synchronize(0)
    driver.Synchronize(0)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    realtime_timer.Spin(step_size)