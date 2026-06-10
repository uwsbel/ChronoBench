import os
import math
import numpy as np                                          # obstacle placement RNG
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())        # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')    # locate vehicle data files

step_size = 2e-3                                            # integration step (SCM needs small step)
tire_step_size = 1e-3                                       # tire force-model substep
sim_end = 12.0                                              # total simulated time (s)

init_loc = chrono.ChVector3d(-5, 0, 0.6)                   # spawn near low edge of the hill
init_rot = chrono.QuatFromAngleZ(0)                         # facing +X, into the bump

hmmwv = veh.HMMWV_Full()                                    # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)         # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)      # no chassis collision against soil
hmmwv.SetChassisFixed(False)                               # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                # SCM needs a non-rigid tire (RIGID won't drive)
hmmwv.SetTireStepSize(tire_step_size)                      # tire substep
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                  # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())      # report total vehicle mass

# SCM deformable hill terrain initialized from the shipped bump heightmap.
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # colored sinkage overlay
terrain.AddMovingPatch(                                     # update only cells near the chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                   40, 40, -1, 1, 0.02)                     # bumpy hill heightmap
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# TMEASY tires need explicit spindle collision cylinders so SCM rays detect sinkage.
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
TIRE_FAMILY = 1
for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)

# Five box obstacles randomly positioned across the terrain.
rng = np.random.default_rng(42)                            # deterministic obstacle layout
obstacle_mat = chrono.ChContactMaterialSMC()
obstacle_mat.SetFriction(0.8)
obstacle_mat.SetRestitution(0.01)
for i in range(5):
    bx = float(rng.uniform(0, 15))                         # ahead of the spawn point
    by = float(rng.uniform(-8, 8))
    box = chrono.ChBodyEasyBox(0.6, 0.6, 0.6, 800, True, True, obstacle_mat)
    box.SetPos(chrono.ChVector3d(bx, by, 1.0))             # drop onto the hill surface
    box.SetFixed(False)                                    # dynamic obstacle
    system.AddBody(box)

system.GetCollisionSystem().BindAll()                      # rebuild collision models after edits

# Lidar sensor mounted on the vehicle chassis.
manager = sens.ChSensorManager(system)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1.0),                        # forward of the chassis, above the hood
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
horizontal_samples = 800
vertical_samples = 300
lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),             # rides on the chassis
    5.0,                               # update_rate (Hz)
    offset_pose,                        # offset pose on the chassis
    horizontal_samples,                 # h_samples
    vertical_samples,                   # v_samples
    2 * chrono.CH_PI,                  # horizontal_fov (rad)
    chrono.CH_PI / 12,                 # max_vert_angle
    -chrono.CH_PI / 6,                 # min_vert_angle
    100.0,                             # max_range
    sens.LidarBeamShape_RECTANGULAR,    # beam shape
    2,                                 # sample_radius
    0.003,                             # vert divergence_angle
    0.003,                             # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)                       # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())                  # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())               # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())                # host access to XYZI
manager.AddSensor(lidar)

# Vehicle-specific Irrlicht visualization.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM hill with lidar")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                         # s to reach full steering
throttle_time = 1.0                                        # s to reach full throttle
braking_time = 0.3                                         # s to reach full brake
render_step_size = 1.0 / 50.0
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

render_steps = math.ceil(render_step_size / step_size)     # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    time = hmmwv.GetSystem().GetChTime()

    if step_number % render_steps == 0:                    # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                               # advances the wrapper-owned system
    vis.Advance(step_size)

    manager.Update()                                       # update the sensor manager once per step


    step_number += 1
    realtime_timer.Spin(step_size)                         # spin so wall-clock matches sim time
