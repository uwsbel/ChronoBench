import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import math as m

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change material properties based on location
var_materials : bool = False

# Callback function to change material properties
def my_material_callback(material, loc) :
    if loc.y > 0 :
        material.SetColor(irr.SColor(0xff, 0xff, 0xff, 0x80))
    else :
        material.SetColor(irr.SColor(0xff, 0x80, 0x80, 0x80))

# Initial system momentum (optional)
#momentum = chrono.ChVector3d(0, 0, 0)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir = "./CASCADE"

# =============================================================================

#print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

# --------------
# Create systems
# --------------

# Create the cascade physical system
sys = cascade.ChCascadeSystemSMC()

# Set collision system type
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set global collision margins
sys.SetCollisionMargin(0.001)

# Define the ground body
ground_mat = chrono.ChContactMaterialSMC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = sys.AddBody(chrono.ChBodyEasyBox(1000, 1000, 1, 1000, True, True, ground_mat))
ground.SetPos(chrono.ChVector3d(0,0,-1))
ground.SetFixed(True)
sys.GetCollisionSystem().AddBox(ground, 1000, 1000, 1)

# Define the left moving wall (will be driven by a motor)
wall_mat = chrono.ChContactMaterialSMC()
wall_mat.SetFriction(0.8)
wall_mat.SetRestitution(0.0)
wall = sys.AddBody(chrono.ChBodyEasyBox(1, 1000, 50, 1000, True, True, wall_mat))
wall.SetPos(chrono.ChVector3d(-10,0,25))
wall.SetRot(chrono.ChQuaterniond(1,0,0,0))
sys.GetCollisionSystem().AddBox(wall, 1, 1000, 50)

# Define the right wall (will be driven by a motor)
wall2 = sys.AddBody(chrono.ChBodyEasyBox(1, 1000, 50, 1000, True, True, wall_mat))
wall2.SetPos(chrono.ChVector3d(10,0,25))
wall2.SetRot(chrono.ChQuaterniond(1,0,0,0))
sys.GetCollisionSystem().AddBox(wall2, 1, 1000, 50)

# Init the visualization system
vis = cascade.ChCascadeVisualSystemIrrlicht()
vis.SetWindowTitle('SMC')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachSystem(sys)

# Create the driver system
driver = cascade.ChCascadeDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# Initialize the driver system.
driver.Initialize()

# ---------------
# Simulation loop
# ---------------

# Number of simulation steps between miscellaneous events
render_steps = m.ceil(render_step_size / step_size)

# Initialize simulation frame counter s
step_number = 0
render_frame = 0

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

sys.GetSolver().AsIterative().SetMaxIterations(150)

sys.Initialize()

# Output exodus data
if (out_dir != "./CASCADE"):
    vis.SetOutputPath(out_dir)
    driver.SetOutputPath(out_dir)
    sys.SetOutputPath(out_dir)
    print("Output path: ", out_dir , "\n")

vis.BeginScene()
vis.Render()
vis.EndScene()

# Initialize output
if (var_materials):
    print("Materials will change based on location\n")

# ---------------
# Simulation loop
# ---------------

# Real time timer
timer = chrono.ChRealtimeStepTimer()
while vis.Run() :
    time = sys.GetChTime()

    # End simulation
    if (time >= tend):
        break

    # Render scene
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    sys.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    vis.Advance(step_size)
    sys.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    timer.Spin(step_size)