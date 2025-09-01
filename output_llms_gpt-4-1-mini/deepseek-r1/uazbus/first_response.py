import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import time

# 1. Initialize PyChrono environment
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Add physical systems and objects

# Create terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 0, 1), 
                        100, 100)
terrain.Initialize()

# Create and initialize UAZ vehicle
uaz = veh.UAZBUS()
uaz.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
uaz.Initialize(system)
uaz.SetChassisFixed(False)
uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)

# 3. Set parameters and systems

# Initialize driver system
driver = veh.ChInteractiveDriverIRR(uaz.GetVehicle())
steering_controller = veh.ChSteeringController()
driver.SetSteeringController(steering_controller)

# Set driver parameters
driver.SetThrottleGain(0.4)
driver.SetSteeringGain(0.3)
driver.SetBrakingGain(0.3)

# Create PID controller for speed control
pid = veh.ChSpeedController()
uaz.GetVehicle().SetSpeedController(pid)
pid.SetGains(0.4, 0, 0)

# 4. Visualization setup
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('UAZBUS Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-5, 0, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddLightWithShadow(chrono.ChVectorD(-5, 0, 5), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 35, 512, chrono.ChColor(1, 1, 1))
vis.EnableShadows()

# 5. Simulation loop parameters
step_size = 0.005
realtime_step = 0.01
time_total = 0

# Attach vehicle to visualization
uaz.GetVehicle().AttachVisualSystem(vis)

# Main simulation loop
while vis.Run():
    time_current = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    uaz.GetVehicle().Synchronize(time_current, driver_inputs, terrain)
    terrain.Synchronize(time_current)
    
    # Advance simulation
    driver.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time execution
    time.sleep(max(realtime_step - system.GetChTime() + time_total, 0))
    time_total = system.GetChTime()

print("Simulation completed.")