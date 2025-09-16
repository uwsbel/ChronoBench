import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Time interval between two render frames
render_step_size = 1 / 50  # FPS = 50

# Simulation end time
t_end = 30

# =============================================================================
# Create the simulation system and add items
# =============================================================================

# Create the Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the vehicle system
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetInitFwdVel(5)
vehicle.Initialize()

# Add the vehicle to the system
system.Add(vehicle.GetChassisBody())
system.Add(vehicle.GetSystem())

# Create the terrain
terrain = veh.RigidTerrain(system)
terrain.SetContactFrictionCoefficient(0.8)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize(0, 0, 0)

# Create the driver system
driver = veh.ChDriverM113()
driver.Initialize()

# =============================================================================
# Visualization system
# =============================================================================

# Create the Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("M113 Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(8, 5, 3))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(8, 5, 10), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512)
vis.SetShadows(True)
vis.AssetBindAll()
vis.AssetUpdateAll()

# =============================================================================
# Simulation loop
# =============================================================================

# Initialize simulation frame counter
frame = 0

# Simulation loop
while vis.Run():
    # Get current time
    time = system.GetChTime()

    # Exit simulation at specified time
    if time > t_end:
        break

    # Render scene
    if frame % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update driver inputs
    driver.SetThrottle(0.5)
    driver.SetSteering(0.1 * np.sin(2 * np.pi * 0.1 * time))

    # Update the vehicle
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetThrottle(), driver.GetSteering(), driver.GetBraking())
    terrain.Synchronize(time)

    # Advance simulation
    system.DoStepDynamics(step_size)

    # Increment frame counter
    frame += 1