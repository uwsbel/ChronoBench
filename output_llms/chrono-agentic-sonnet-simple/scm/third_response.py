import os
import math
import random
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())           # locate bundled Chrono data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')       # locate vehicle data files

# simulation parameters
step_size = 5e-4                                               # physics step 0.5 ms
sim_end = 30.0                                                 # run for 30 s
render_fps = 50                                                # Irrlicht frame rate
render_every = max(1, round(1.0 / (render_fps * step_size)))  # render cadence (untagged)

# initial vehicle placement
initLoc = chrono.ChVector3d(0, 0, 0.5)                        # spawn above terrain
initRot = chrono.QuatFromAngleZ(0.0)                          # facing +X

# HMMWV on SCM — SMC contact, TMEASY tires (required for SCM driving)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)             # SMC for SCM terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                   # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                   # TMEASY needed on SCM (RIGID won't drive)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

system = hmmwv.GetSystem()                                     # get the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())         # truth's literal mass banner

# visualization types after Initialize
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# SCM terrain setup
terrain = veh.SCMTerrain(system)                               # deformable soft soil
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi  — frictional modulus (Pa)
    0,      # Bekker_Kc    — cohesive modulus
    1.1,    # Bekker_n     — exponent
    0,      # Mohr_cohesion
    30,     # Mohr_friction angle (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K (Pa/m)
    3e4,    # damping_R (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)    # sinkage heatmap

terrain.AddMovingPatch(                                        # moving patch for performance
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                               # patch centred on chassis
    chrono.ChVector3d(5, 3, 1),                               # patch extent (truth's values)
)
terrain.Initialize(120.0, 120.0, 0.1)                         # 120x120 m, 0.1 m grid
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,                                                    # UV tiling
)

# tire collision cylinders — required for TMEASY on SCM
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w   = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY    = 1
SUPPORT_FAMILY = 4
CHASSIS_FAMILY = 3

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
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)              # tires don't collide with each other
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)           # tires ride on SCM, not support

# hidden rigid support plane — props fall through SCM surface otherwise
support_mat = chrono.ChContactMaterialSMC()
support_mat.SetFriction(0.9)
support_mat.SetRestitution(0.01)
support_mat.SetYoungModulus(2e7)
support = chrono.ChBodyEasyBox(120.0, 120.0, 0.2, 1000, False, True, support_mat)
support.SetName("asset_support_ground")
support.SetPos(chrono.ChVector3d(0, 0, -0.1))                 # top at z=0 (SCM rest plane)
support.SetFixed(True)
support.EnableCollision(True)
support_cm = support.GetCollisionModel()
support_cm.SetFamily(SUPPORT_FAMILY)
support_cm.DisallowCollisionsWith(TIRE_FAMILY)                 # tires ride on SCM only
support_cm.DisallowCollisionsWith(CHASSIS_FAMILY)
system.AddBody(support)

# randomly positioned boxes — not within the vehicle footprint
random.seed(42)                                                # deterministic placement
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.6)
box_mat.SetRestitution(0.05)
box_mat.SetYoungModulus(2e7)

num_boxes = 20                                                 # number of scene boxes
veh_x = initLoc.x
veh_y = initLoc.y
exclusion_r = 4.0                                              # clear radius around vehicle spawn

for _ in range(num_boxes):
    bx = by = 0.0
    for _attempt in range(200):                               # rejection-sample far from vehicle
        bx = random.uniform(-40, 40)
        by = random.uniform(-40, 40)
        if math.sqrt((bx - veh_x)**2 + (by - veh_y)**2) > exclusion_r:
            break
    bw = random.uniform(0.3, 1.0)                             # box half-width
    bh = random.uniform(0.3, 0.8)                             # box half-height
    box = chrono.ChBodyEasyBox(
        bw * 2, bw * 2, bh * 2,                              # full dimensions
        400,                                                   # density
        True,                                                  # visualize
        True,                                                  # collide
        box_mat,
    )
    box.SetPos(chrono.ChVector3d(bx, by, bh))                 # rest on terrain surface (z=0)
    box.SetFixed(False)                                        # dynamic props
    system.AddBody(box)

# rebuild all collision models after adding shapes
system.GetCollisionSystem().BindAll()

# sensor manager with point lights
manager = sens.ChSensorManager(system)                         # create sensor manager
intensity = 1.0
manager.scene.AddPointLight(                                   # light 1 — above-left
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                   # light 2 — forward
    chrono.ChVector3f(10, 10, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                   # light 3 — side
    chrono.ChVector3f(-10, 10, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# camera sensor attached to the vehicle chassis
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 2),                              # behind and above chassis origin
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)), # slight downward tilt
)
cam_res_w, cam_res_h = 1280, 720                              # camera resolution
cam_fov = 1.408                                               # horizontal FOV ~80 deg
cam = sens.ChCameraSensor(
    hmmwv.GetChassisBody(),                                   # attach to chassis body
    30,                                                        # update_rate Hz (physical)
    cam_offset,
    cam_res_w,
    cam_res_h,
    cam_fov,
)
cam.SetName("Chassis Camera")
cam.SetLag(0)                                                  # no lag
cam.SetCollectionWindow(0)                                     # snapshot exposure

# filter chain (ORDER MATTERS — scored core, never review-only)
cam.PushFilter(sens.ChFilterVisualize(cam_res_w, cam_res_h, "RGB Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                    # host buffer access
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))         # save PNG frames to disk

manager.AddSensor(cam)                                        # register after pushing all filters

# check for buffer data (scored sensor diagnostic)
buf = cam.GetMostRecentRGBA8Buffer()
if buf.HasData():
    print('Buffer received. Resolution: {0}x{1}'.format(buf.Width, buf.Height))

# Irrlicht vehicle visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM + Sensor")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)  # chase camera
vis.Initialize()                                               # INITIALIZE FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                      # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# interactive driver
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * step_size / 1.0)       # time to full steer = 1 s
driver.SetThrottleDelta(render_every * step_size / 1.0)       # time to full throttle = 1 s
driver.SetBrakingDelta(render_every * step_size / 0.3)        # time to full brake = 0.3 s
driver.Initialize()


step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()                 # real-time pacing

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    if step_number % render_every == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)            # 3-arg for wheeled
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                   # advances the wrapper-owned system
    vis.Advance(step_size)

    manager.Update()                                           # pump all sensors once per step


    step_number += 1
    realtime_timer.Spin(step_size)                             # pace to wall clock
