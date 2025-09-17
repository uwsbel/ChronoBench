import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.hmmwv as hmmwv
import pychrono.irrlicht as irr
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 0.01

# Simulation end time
t_end = 30

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.0)

# =============================================================================
# Create the systems
# =============================================================================

# Create the vehicle system
vehicle = veh.Vehicle()
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetInitFwdVel(5)

# Create and initialize the HMMWV vehicle
hmmwv.Init(vehicle)

# Create the SCM deformable terrain
terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetPlane(chrono.ChVectorD(0, 0, 1), 0)
terrain.SetSoilParameters(2e6,  # Bekker Kphi
                          0,    # Bekker Kc
                          1.1,  # Bekker n exponent
                          0.0,  # Mohr cohesive limit (Pa)
                          30,   # Mohr friction limit (degrees)
                          0.01) # Janosi shear coefficient (m)
terrain.EnablePressureSinkage(True)
terrain.EnableRigidBase(True)
terrain.SetRigidBaseHeight(-0.5)
terrain.SetRigidBaseFriction(0.8)

# Create a height map for the terrain
terrain.Initialize(10, 10, 20, 20)  # nx, ny, dx, dy

# Create the driver system
driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetTargetSpeed(5)

# =============================================================================
# Create the visualization system
# =============================================================================

# Create the Irrlicht visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(vehicle.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, -3, 2), trackPoint)
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 20, 1, 20, 40, 512)
vis.SetShadows(True)

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps between two 3D view render frames
render_steps = 1

# Initialize simulation frame counter
step_number = 0

# Main simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Exit simulation when end time is reached
    if time >= t_end:
        break

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update driver inputs
    driver.Synchronize(time)
    driver.SetSteering(0.2 * np.sin(0.5 * time))  # Example steering input

    # Update vehicle
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    # Update terrain
    terrain.Synchronize(time)

    # Advance simulation for one timestep for all modules
    vehicle.GetSystem().DoStepDynamics(step_size)

    # Update visualization
    vis.Synchronize(chrono.ChVectorD(vehicle.GetChassisBody().GetPos().x,
                                    vehicle.GetChassisBody().GetPos().y,
                                    vehicle.GetChassisBody().GetPos().z),
                    chrono.ChQuaternionD(1, 0, 0, 0))

    # Increment frame number
    step_number += 1

# =============================================================================
# Clean up
# =============================================================================

# Close the visualization
vis.Close()