import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts — PRIMITIVES (simplified from mesh, per prompt)
vis_type = veh.VisualizationType_PRIMITIVES

# Chassis collision type — PRIMITIVES (simple primitive shapes, per prompt)
chassis_collision_type = veh.CollisionType_PRIMITIVES

# Tire model
tire_model = veh.TireModelType_TMEASY

# Rigid terrain dimensions
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames (50 FPS)
render_step_size = 1.0 / 50

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)         # NSC for rigid terrain
vehicle.SetChassisCollisionType(chassis_collision_type)       # PRIMITIVES collision shape
vehicle.SetChassisFixed(False)                                # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization type to PRIMITIVES for all vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED

# Output total vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Create the rigid terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Driver time response — made LESS responsive (larger times = slower ramp-up, per prompt)
steering_time = 4.0   # time to go from 0 to +1 steering (increased from 1.0 → slower)
throttle_time = 4.0   # time to go from 0 to +1 throttle (increased from 1.0 → slower)
braking_time = 1.0    # time to go from 0 to +1 braking (increased from 0.3 → slower)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)                                    # real-time pacing
