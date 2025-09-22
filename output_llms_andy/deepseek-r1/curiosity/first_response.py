import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr
from pychrono import ChVectorD, ChQuaternionD, ChCoordsysD

# =============================================================================
# Main simulation script
# =============================================================================

# Set path to Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), ''))

# Create physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(ChVectorD(0, -9.81, 0))

# ------------------------
# Create Rigid Terrain
# ------------------------
terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
terrain.SetPos(ChVectorD(0, -1, 0))

# Collision properties
terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.8)
terrain_mat.SetRestitution(0.1)

terrain.GetCollisionModel().ClearModel()
terrain.GetCollisionModel().AddBox(terrain_mat, 100, 1, 100)  # Size (x,y,z)
terrain.GetCollisionModel().BuildModel()
terrain.SetCollide(True)

# Visualization
terrain_shape = chrono.ChBoxShape()
terrain_shape.GetBoxGeometry().Size = ChVectorD(100, 1, 100)
terrain_shape.SetTexture(chrono.GetChronoDataFile('textures/checker2.png'), 200, 200)
terrain.AddAsset(terrain_shape)

system.Add(terrain)

# ------------------------
# Create Curiosity Rover
# ------------------------
# Add rover Python module to path
rover_path = os.path.join(chrono.GetChronoDataPath(), 'robot/curiosity')
if rover_path not in sys.path:
    sys.path.append(rover_path)
from curiosity import Curiosity

# Initialize rover
rover = Curiosity()
rover.SetDriverInputs([0]*6, [0]*6)  # Initialize with neutral inputs

# Position rover above terrain
init_pos = ChCoordsysD(ChVectorD(0, 0, 0), 
                       ChQuaternionD(1, 0, 0, 0))
rover.Initialize(init_pos)
system.Add(rover.GetChassis())

# ------------------------
# Visualization Setup
# ------------------------
app = chronoirr.ChIrrApp(system, 
                         "Curiosity Rover Simulation", 
                         chronoirr.dimension2du(1280, 720),
                         False,  # Fullscreen?
                         True)   # Shadows?

# Configure visualization
app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.AddSkyBox()
app.AddTypicalLights(chronoirr.vector3df(30, 100, 30), 
                    chronoirr.vector3df(30, -80, -30), 
                    250, 130)
app.AddLightWithShadow(chronoirr.vector3df(10, 25, 10), 
                      chronoirr.vector3df(0, 0, 0), 
                      50, 5, 40, 85, 512)

# Camera settings
camera = app.AddCamera(chronoirr.vector3df(3, 2, 3), 
                      chronoirr.vector3df(0, 0, 0))
camera.SetTarget(ChVectorD(0, 0, 0))
app.AddTypicalCamera(chronoirr.vector3df(8, 4, -6))

# Shadows and effects
app.AddShadowAll()
app.EnableShadows(True)

# Finalize asset construction
app.AssetBindAll()
app.AssetUpdateAll()

# ------------------------
# Driver Controls Setup
# ------------------------
driver = chrono.ChIrrGuiDriver(app)

# Steering parameters
max_steering = 0.5  # Max steering angle (rad)
max_speed = 10      # Max wheel rotation speed (rad/s)

# Simulation parameters
time_step = 0.001
app.SetTimestep(time_step)
app.SetTryRealtime(True)

# =============================================================================
# Simulation Loop
# =============================================================================
while app.GetDevice().run():
    # Process GUI inputs
    driver.ProcessInputs()
    
    # Get current control inputs
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking = driver.GetBraking()
    
    # Convert to rover inputs
    steering_inputs = [
        steering * max_steering, -steering * max_steering,
        steering * max_steering, -steering * max_steering,
        steering * max_steering, -steering * max_steering
    ]
    drive_inputs = [(throttle - braking) * max_speed] * 6
    
    # Apply to rover
    rover.SetDriverInputs(steering_inputs, drive_inputs)
    
    # Update camera to follow rover
    rover_pos = rover.GetChassis().GetPos()
    camera.SetPosition(chronoirr.vector3df(
        rover_pos.x + 3, rover_pos.y + 2, rover_pos.z + 3))
    camera.SetTarget(chronoirr.vector3df(
        rover_pos.x, rover_pos.y, rover_pos.z))
    
    # Advance simulation
    app.BeginScene(True, True, chronoirr.SColor(100, 101, 140, 255))
    app.DrawAll()
    app.DoStep()
    app.EndScene()