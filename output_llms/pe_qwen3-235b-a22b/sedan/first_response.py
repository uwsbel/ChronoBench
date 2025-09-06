import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.driver as chronodriv

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))

# Create the system using Smooth Contact (SMC)
sys = chrono.ChSystemSMC()

# Configure collision detection
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create rigid terrain with customizable texture
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Add terrain patch with concrete texture
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         200, 100)  # Length, Width
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()

# Create BMW E90 Sedan (using HMMWV as placeholder - replace with actual E90 model if available)
car = veh.WheeledVehicle(sys, chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv.json"))
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
car.SetTireType(veh.TireModelType_TMEASY)  # TMEASY tire model
car.SetChassisCollisionType(veh.ChassisCollisionType_ALL)  # Enable collision
car.Initialize()

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('BMW E90 Sedan Simulation - TMEASY Tires on Rigid Terrain')
vis.Initialize()

# Configure visualization assets
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -5))  # Initial camera position
vis.AddTypicalLights(chrono.ChVector3d(30, 50, 30),  # Light position
                     chrono.ChColor(0.7, 0.7, 0.7),  # Ambient
                     chrono.ChColor(0.9, 0.9, 0.9))  # Diffuse

# Create interactive driver system
driver = chronodriv.ChIrrGuiDriver(vis.GetDevice())
driver.SetSteeringDelta(0.05)   # Max steering change per step
driver.SetThrottleDelta(0.05)  # Max throttle change per step
driver.SetBrakingDelta(0.05)   # Max braking change per step
driver.Initialize()

# Simulation loop
time_step = 0.01  # 100Hz simulation rate

while vis.Run():
    # Get current simulation time
    time = sys.GetChTime()
    
    # Synchronize components
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    car.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    sys.DoStepDynamics(time_step)
    
    # Update chase camera
    chassis = car.GetChassis()
    chassis_pos = chassis.GetPos()
    chassis_rot = chassis.GetRot()
    
    # Chase camera offset (follows behind the vehicle)
    chase_offset = chrono.ChVector3d(0, 1.5, -4)
    chase_offset = chassis_rot.Rotate(chase_offset)
    
    cam_pos = chassis_pos + chase_offset
    cam_target = chassis_pos + chrono.ChVector3d(0, 0.5, 0)
    
    vis.SetCameraPosition(cam_pos, cam_target)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()