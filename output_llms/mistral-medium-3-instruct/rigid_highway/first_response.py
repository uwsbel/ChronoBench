import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.wheeled_vehicle as wheeled
import pychrono.vehicle.hmmwv as hmmwv
import pychrono.vehicle.terrain as terrain
import pychrono.vehicle.driver as driver
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 0.02  # 50 Hz

# Simulation end time
t_end = 100

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.0)

# =============================================================================
# Create the systems and set solver settings
# =============================================================================

# Create the vehicle system
vehicle = veh.Vehicle()
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollide(True)
vehicle.SetInitializeFwdVel(10)
vehicle.SetInitializePos(chrono.ChVectorD(0, 0, 1.0))

# Create and initialize the HMMWV vehicle
hmmwv.Init()

# Create the HMMWV vehicle, set parameters, and initialize
hmmwv = hmmwv.HMMWV(vehicle)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)
hmmwv.SetChassisFixed(False)
hmmwv.SetChassisCollide(True)
hmmwv.SetChassisMass(2000)
hmmwv.SetChassisInertiaXX(chrono.ChVectorD(1000, 1000, 1000))
hmmwv.SetChassisCOM(chrono.ChVectorD(0, 0, 0.5))
hmmwv.SetDriverType(driver.DriverType_INTERACTIVE)
hmmwv.SetDriverVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.Initialize()

# Set solver settings
hmmwv.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
hmmwv.GetSystem().SetMaxIterationNormal(100)
hmmwv.GetSystem().SetMaxIterationSliding(100)
hmmwv.GetSystem().SetMaxIterationBilateral(100)
hmmwv.GetSystem().SetTolForce(1e-3)

# =============================================================================
# Create the terrain
# =============================================================================

# Create the terrain system
terrain_system = terrain.RigidTerrain(hmmwv.GetSystem())

# Load collision and visual meshes
collision_mesh = chrono.ChTriangleMeshConnected()
collision_mesh.LoadWavefrontMesh("Highway_col.obj", True, True)
visual_mesh = chrono.ChTriangleMeshConnected()
visual_mesh.LoadWavefrontMesh("Highway_vis.obj", True, True)

# Create the terrain body
terrain_body = chrono.ChBody()
terrain_body.SetBodyFixed(True)
terrain_body.SetCollide(True)
terrain_body.GetCollisionModel().AddTriangleMesh(collision_mesh)
terrain_body.GetVisualModel().AddTriangleMesh(visual_mesh)
terrain_body.SetPos(chrono.ChVectorD(0, 0, 0))

# Add terrain body to the system
hmmwv.GetSystem().AddBody(terrain_body)

# =============================================================================
# Create the Irrlicht application
# =============================================================================

# Create the Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()
vis.AttachSystem(hmmwv.GetSystem())

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps between two 3D view render frames
render_steps = int(np.ceil((1.0 / 50) / step_size))

# Initialize simulation frame counter
step_number = 0

# Simulation loop
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Exit simulation loop when end time is reached
    if time >= t_end:
        break

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update driver inputs
    driver_inputs = hmmwv.GetDriver().GetInputs()
    driver_inputs.m_steering = vis.GetDevice()->getEventReceiver().GetSteering()
    driver_inputs.m_throttle = vis.GetDevice()->getEventReceiver().GetThrottle()
    driver_inputs.m_braking = vis.GetDevice()->getEventReceiver().GetBraking()
    hmmwv.GetDriver().SetInputs(driver_inputs)

    # Update modules (process inputs from other modules)
    hmmwv.Synchronize(time)
    hmmwv.Update(time)

    # Advance simulation for one timestep for all modules
    hmmwv.Advance(step_size)

    # Increment frame number
    step_number += 1

# Clean up
vis.CloseDevice()