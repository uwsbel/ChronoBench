import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import os

# =============================================================================
# Simulation Parameters
# =============================================================================
TIME_STEP = 0.001
SIMULATION_TIME = 30.0
TERRAIN_SIZE = 10.0  # meters
TERRAIN_HEIGHT = 0.2  # meters
ROVER_INIT_POS = chrono.ChVectorD(0, 1.5, 0)
CAMERA_POS = chrono.ChVectorD(3, 2, 3)  # Camera position relative to rover

# =============================================================================
# Initialize Simulation
# =============================================================================
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity
system.SetSolverMaxIterations(100)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# =============================================================================
# Create Rigid Terrain
# =============================================================================
ground = chrono.ChBodyEasyBox(TERRAIN_SIZE, TERRAIN_HEIGHT, TERRAIN_SIZE, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -TERRAIN_HEIGHT/2, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# =============================================================================
# Create Curiosity Rover
# =============================================================================
rover = veh.Curiosity(system)
rover.Initialize(chrono.ChFrameD(ROVER_INIT_POS))
rover.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

# Configure motor control
motor_speed = 0.3  # rad/s
steering_speed = 0.05  # rad/s
driver = rover.GetDriver()
driver.SetDesiredSpeed(motor_speed)
driver.SetSteering(steering_speed)

# =============================================================================
# Visualization Setup
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAMERA_POS, ROVER_INIT_POS)
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 8, -5), ROVER_INIT_POS, 15, 2, 10, 40, 512)

# Enable shadows
vis.EnableShadows()

# =============================================================================
# Simulation Loop
# =============================================================================
time = 0.0
while vis.Run() and time < SIMULATION_TIME:
    vis.BeginScene()
    vis.Render()
    vis.RenderFrame(ROVER_INIT_POS, chrono.QUNIT, 1.0)  # Reference frame
    vis.EndScene()
    
    # Update rover controls
    driver.Update(chrono.ChTimer())
    
    # Advance simulation
    system.DoStepDynamics(TIME_STEP)
    time += TIME_STEP

    # Update camera position to follow rover
    cam_pos = rover.GetChassis().GetPos() + CAMERA_POS
    vis.GetActiveCamera().setTarget(chronoirr.vector3df(
        rover.GetChassis().GetPos().x,
        rover.GetChassis().GetPos().y,
        rover.GetChassis().GetPos().z))