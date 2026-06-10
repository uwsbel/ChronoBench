import os
import math
import numpy as np                                                   # random obstacle placement
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire substep (s)
init_loc = chrono.ChVector3d(-5, 0, 0.6)                             # spawn near hill base
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X

hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                   # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision mesh
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # SCM needs TMEASY, not RIGID
hmmwv.SetTireStepSize(tire_step_size)                                # tire integration substep
hmmwv.Initialize()                                                   # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tire mesh

system = hmmwv.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

# SCM deformable soft-soil terrain initialized from a bump heightmap (the "hill").
terrain = veh.SCMTerrain(system)                                     # deformable Bekker-Wong terrain
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent (soft soil)
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa.s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)          # colored sinkage overlay
terrain.AddMovingPatch(                                              # only refine cells near the chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                      # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                      # OOBB dims (m) — truth uses (5,3,1)
)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                   40, 40, -1, 1, 0.02)                              # heightmap, length, width, hMin, hMax, res
terrain.SetMeshWireframe(False)                                      # solid mesh
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

# TMEASY tires carry no auto collision geometry — add a collision cylinder to each spindle
# so SCM ray-casts detect sinkage and ruts form.
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()                            # SMC contact material for tires
tire_mat.SetFriction(0.9)                                           # tire friction
tire_mat.SetRestitution(0.1)                                        # tire restitution

TIRE_FAMILY = 1                                                      # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()                    # spindle body
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),  # +4cm to ensure sinkage
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)                               # enable spindle collision
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)                                # tag family
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)                  # tires never collide with each other

# Five box obstacles at random positions scattered on the terrain.
np.random.seed(0)                                                   # deterministic obstacle layout
obstacle_mat = chrono.ChContactMaterialSMC()                       # SMC material for obstacles
obstacle_mat.SetFriction(0.9)                                       # obstacle friction
obstacle_mat.SetRestitution(0.01)                                  # obstacle restitution
obstacle_mat.SetYoungModulus(2e7)                                  # obstacle stiffness
for i in range(5):                                                  # 5 random box obstacles
    bx = np.random.uniform(-15, 15)                                # random X within terrain
    by = np.random.uniform(-15, 15)                                # random Y within terrain
    box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, True, obstacle_mat)  # 1m cube, dynamic
    box.SetPos(chrono.ChVector3d(bx, by, 1.0))                     # drop onto the terrain
    box.SetFixed(False)                                            # dynamic obstacle
    system.AddBody(box)                                            # register with the system

# Sensor manager oversees the lidar.
manager = sens.ChSensorManager(system)                             # owns all sensors on this system

# Lidar mounted on the vehicle chassis, scanning the surroundings (incl. obstacles).
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.5, 0, 1.0),                                # mount above/front of chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),       # no tilt
)
horizontal_samples = 800                                            # horizontal beams
vertical_samples = 300                                              # vertical channels
lidar = sens.ChLidarSensor(
    hmmwv.GetChassisBody(),                                         # attached to the chassis
    5.0,                                                           # update_rate (Hz)
    offset_pose,                                                    # mount pose
    horizontal_samples,                                            # h_samples
    vertical_samples,                                              # v_samples
    2 * chrono.CH_PI,                                              # horizontal_fov (rad)
    chrono.CH_PI / 12,                                            # max_vert_angle (rad)
    -chrono.CH_PI / 6,                                            # min_vert_angle (rad)
    100.0,                                                         # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,                              # beam shape
    2,                                                            # sample_radius
    0.003,                                                        # vert divergence_angle
    0.003,                                                        # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,                        # strongest return
)
lidar.SetName("Lidar Sensor")                                      # name
lidar.SetLag(0)                                                    # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                              # collection window = 1/update_rate

# Lidar filter chain (ORDER MATTERS) — scored core, never review-only.
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())                          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                      # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())                       # host access to XYZI
manager.AddSensor(lidar)                                           # register the lidar

# Interactive driver bound to the vehicle visual system.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle-specific Irrlicht system
vis.SetWindowTitle("HMMWV on SCM Hill with Lidar")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)       # chase camera on chassis
vis.Initialize()                                                  # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                         # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                             # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                      # 50 FPS rendering
render_steps = math.ceil(render_step_size / step_size)            # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive driver on the vis
driver.SetSteeringDelta(render_step_size / 1.0)                   # 0->1 steering in 1 s
driver.SetThrottleDelta(render_step_size / 1.0)                   # 0->1 throttle in 1 s
driver.SetBrakingDelta(render_step_size / 0.3)                    # 0->1 brake in 0.3 s
driver.Initialize()                                               # init driver

system.GetCollisionSystem().BindAll()                            # rebuild collision models after spindle cylinders

render_every = render_steps                                       # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                     # real-time pacing
step_number = 0
while vis.Run():
    time = system.GetChTime()                                     # current sim time

    if step_number % render_steps == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver inputs

    driver.Synchronize(time)                                     # sync driver
    terrain.Synchronize(time)                                    # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)             # sync vehicle (3-arg, wheeled)
    vis.Synchronize(time, driver_inputs)                        # sync visualization

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    hmmwv.Advance(step_size)                                     # advances the wrapper-owned system
    vis.Advance(step_size)                                       # advance visualization

    manager.Update()                                             # update the sensor manager (lidar) once per step


    step_number += 1
    realtime_timer.Spin(step_size)                              # spin so wall-clock matches sim time

    if time > 12.0:                                             # cap the record run
        break
