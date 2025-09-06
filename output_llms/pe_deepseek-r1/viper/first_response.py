import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# ========================
# 1. System Initialization
# ========================
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# =====================
# 2. Create Rigid Terrain
# =====================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         100, 100)  # 100m x 100m terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()

# =====================
# 3. Initialize Viper Rover
# =====================
rover = veh.Viper(system)
rover.SetDriverPos(chrono.ChVector3d(0, 0, 0.5))  # Driver eye position
rover.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), 
                                   chrono.QuatFromAngleY(0)))

# Create and configure driver
driver = veh.ViperSpeedDriver()
driver.SetThrottle(0.3)  # Constant forward throttle
rover.SetDriver(driver)

# =====================
# 4. Visualization Setup
# =====================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(5, 5, 5), chrono.ChVector3d(0, 0, 0.5), 
                       10, 2, 10, 40, 512, chrono.ChColor(0.8, 0.8, 0.8))

# Add rover visualization
rover.InitializeVisualization(vis)

# =====================
# 5. Simulation Parameters
# =====================
step_size = 0.001
end_time = 30.0  # seconds
steering_period = 10.0  # Steering oscillation period
max_steering = 0.5  # Max steering input (-1.0 to 1.0)

# =====================
# 6. Simulation Loop
# =====================
time = 0.0
while time < end_time:
    # Update steering based on time (sine wave pattern)
    steering = max_steering * math.sin(2 * chrono.CH_PI * time / steering_period)
    driver.SetSteering(steering)
    
    # Update rover systems
    rover.Update()
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    time += step_size

    # Display real-time data
    pos = rover.GetChassis().GetPos()
    speed = rover.GetSpeed()
    vis.GetGUIEnvironment().addStaticText(
        f"Time: {time:.2f}s | Steering: {steering:.2f} | Speed: {speed:.2f} m/s | Position: ({pos.x:.2f}, {pos.y:.2f})",
        chronoirr.recti(10, 10, 400, 40))

# For uneven terrain:
   terrain.EnableMeshCreation(True)
   terrain.SetMeshResolution(0.5)  # Higher = smoother

# Modify rover properties:
   rover.SetWheelContactMaterial(chrono.ChContactMaterialSMC())
   rover.SetChassisVisualizationType(veh.VisualizationType_MESH)

# For linear steering ramp:
   steering = max_steering * min(time / 5.0, 1.0)  # Ramp up over 5 seconds

# Add after vis.BeginScene():
   vis.CameraFollow(rover.GetChassisBody(), chrono.ChVector3d(0, 5, 2), 5.0)