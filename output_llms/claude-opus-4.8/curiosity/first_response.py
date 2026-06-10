import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math

# ---------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core system
# ---------------------------------------------------------------------

# Set the path to the Chrono data directory (textures, models, etc.)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Create the physical system (with Non-Smooth Contacts)
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------------------------------------------------------------
# 2. Add the required physical systems and objects
# ---------------------------------------------------------------------

# --- Define a contact material for the ground ---
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.0)

# --- Create the fixed rigid ground (terrain) ---
ground = chrono.ChBodyEasyBox(
    20, 20, 1,        # dimensions (x, y, z)
    1000,             # density
    True,             # visualization enabled
    True,             # collision enabled
    ground_mat        # contact material
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)

# Apply a texture to the ground for visualization
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)

# ---------------------------------------------------------------------
# Create and configure the Curiosity rover
# ---------------------------------------------------------------------

# Instantiate the Curiosity rover model
rover = robot.Curiosity(system)

# Create a driver for motor control / steering inputs
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Define the rover's initial position and orientation
init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# Initialize the rover into the simulation
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# ---------------------------------------------------------------------
# 3. Set up the Irrlicht visualization system
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover on Rigid Terrain')
vis.Initialize()

# Add a logo and skybox to enhance the scene
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Customizable camera settings
vis.AddCamera(chrono.ChVector3d(3, 3, 1.5),   # camera position
              chrono.ChVector3d(0, 0, 0))      # look-at point

# Lighting and shadows
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(5, 5, 10),    # light position
    chrono.ChVector3d(0, 0, 0),     # light target
    20,                             # radius
    1, 15,                          # near/far planes
    55,                             # field of view
    512                             # shadow map resolution
)
vis.EnableShadows()

# ---------------------------------------------------------------------
# Simulation loop with real-time steering inputs
# ---------------------------------------------------------------------

time_step = 1e-3
time = 0.0

while vis.Run():
    time = system.GetChTime()

    # --- Real-time steering control ---
    # Example: apply a gentle steering input as a function of time
    steering = 0.5 * math.sin(time * 0.5)
    driver.SetSteering(steering)

    # Update the rover internal dynamics/controllers
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

print("Simulation finished.")