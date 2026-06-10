import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1.0 / 50  # 50 FPS
tire_step_size = step_size
render_step_size = 1.0 / 50

# Vehicle initial position and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Terrain parameters
terrainHeight = 0
terrainLength = 200.0
terrainWidth = 200.0

# ---------------------
# Initialize the vehicle
# ---------------------
vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set mesh visualization for all vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Retrieve the vehicle system
system = vehicle.GetSystem()

# ---------------------
# Create the terrain
# ---------------------
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# ---------------------
# Create the driver system
# ---------------------
driver = veh.ChInteractiveDriverIRR(vis := chronoirr.ChVisualSystemIrrlicht())

# ---------------------
# Create Irrlicht visualization
# ---------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEDA Vehicle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(-6, -3, 2),   # Camera position
    chrono.ChVector3d(0, 0, 1)       # Camera look-at point
)

# ---------------------
# Create the driver system (re-initialize after vis)
# ---------------------
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# ---------------------
# Simulation loop
# ---------------------
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("Starting FEDA Vehicle Simulation...")
print("Controls:")
print("  Arrow Keys / WASD : Steering and Throttle")
print("  Space             : Braking")
print("  Esc               : Exit")

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene at specified frequency
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update camera to follow the vehicle
    chassis_pos = vehicle.GetChassisBody().GetPos()
    camera_target = chrono.ChVector3d(
        chassis_pos.x,
        chassis_pos.y,
        chassis_pos.z + 1.0
    )
    camera_pos = chrono.ChVector3d(
        chassis_pos.x - 6.0,
        chassis_pos.y - 3.0,
        chassis_pos.z + 2.0
    )
    vis.UpdateCamera(camera_pos, camera_target)

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    # Maintain real-time simulation
    realtime_timer.Spin(step_size)

print("Simulation complete.")