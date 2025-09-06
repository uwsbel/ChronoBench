import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# ======================
# Simulation Parameters
# ======================
contact_method = chrono.ChContactMethod_SMC
step_size = 0.005
tire_type = veh.TireModelType_TMEASY
chassis_collision = False
fix_terrain = True
terrain_length = 100.0
terrain_width = 10.0
skybox_texture = "skybox/"
logo_texture = "logo_pychrono_alpha.png"
camera_trackpoint = chrono.ChVector3d(0.0, 0.0, 1.75)
camera_distance = 6.0
camera_height = 0.5

# ======================
# System Initialization
# ======================
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# ======================
# Vehicle Setup
# ======================
# Create MAN Kat truck (10t variant)
truck = veh.MAN_10t()
truck.SetContactMethod(contact_method)
truck.SetChassisFixed(fix_terrain)
truck.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 0.7), 
    chrono.QUNIT
))
truck.SetTireType(tire_type)
truck.Initialize()

# Configure visualization
truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
truck.SetTireVisualizationType(veh.VisualizationType_MESH)

# ======================
# Terrain Setup
# ======================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 
                         chrono.QUNIT),
                         terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# ======================
# Visualization System
# ======================
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(camera_trackpoint, camera_distance, camera_height)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile(logo_texture))
vis.AddSkyBox()
vis.AddTypicalLights(chrono.ChVector3d(30, -30, 100), chrono.ChVector3d(30, 50, 100), 500, 400)
vis.AddLightDirectional()

# ======================
# Driver Controls Setup
# ======================
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0  # seconds
driver.SetSteeringDelta(0.02 / steering_time)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# ======================
# Simulation Loop
# ======================
truck.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = system.GetChTime()
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update vehicle systems
    truck.GetVehicle().Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation
    system.DoStepDynamics(step_size)
    
    # Update driver
    driver.Synchronize(time)
    
    # Display stats
    vis.Advance(step_size)
    pos = truck.GetVehicle().GetPos()
    speed = truck.GetVehicle().GetSpeed()
    print(f"Time: {time:.2f}s | Position: ({pos.x:.1f}, {pos.y:.1f}) | Speed: {speed * 3.6:.1f} km/h", end='\r')