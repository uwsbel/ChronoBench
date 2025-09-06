import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.kraz as kraz
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Simulation end time
t_end = 10.0

# =============================================================================
# Create the simulation system
# =============================================================================

# Create the Chrono physical system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# =============================================================================
# Create the terrain
# =============================================================================

# Create a rigid terrain body
terrain = chrono.ChBodyEasyBox(20, 20, 0.5, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, 0, -0.5))
terrain.SetBodyFixed(True)
terrain.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
terrain.GetMaterialSurfaceNSC().SetFriction(0.8)
terrain.GetMaterialSurfaceNSC().SetRestitution(0.1)
system.Add(terrain)

# =============================================================================
# Create the Kraz vehicle
# =============================================================================

# Create the Kraz vehicle system
vehicle = kraz.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetInitFwdVel(5)  # Initial forward velocity in m/s
vehicle.Initialize()

# Add the vehicle to the system
system.Add(vehicle.GetVehicle())

# =============================================================================
# Create the driver system
# =============================================================================

# Create a simple driver system
driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
driver.SetSteering(0.0)
driver.SetThrottle(0.5)
driver.SetBraking(0.0)

# =============================================================================
# Visualization system
# =============================================================================

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Kraz Vehicle Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(10, 5, 3), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.SetCameraMoveScale(0.005)
vis.SetCameraMoveSpeed(0.01)
vis.EnableShadows()

# =============================================================================
# Simulation loop
# =============================================================================

# Simulation loop parameters
real_time = True
real_time_factor = 1.0
time_step = step_size
current_time = 0

# Main simulation loop
while vis.Run() and current_time < t_end:
    # Get current time
    start_time = chrono.ChTime.GetTime()

    # Advance simulation for one timestep
    driver.Synchronize(time_step)
    vehicle.Synchronize(time_step, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    system.DoStepDynamics(time_step)
    vehicle.Advance(time_step)

    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update simulation time
    current_time += time_step

    # Control simulation speed (real-time)
    if real_time:
        elapsed_time = chrono.ChTime.GetTime() - start_time
        remaining_time = time_step / real_time_factor - elapsed_time
        if remaining_time > 0:
            chrono.ChTime.MSecSleep(int(remaining_time * 1000))

# =============================================================================
# Clean up
# =============================================================================

# Remove visualization
vis.RemoveAll()