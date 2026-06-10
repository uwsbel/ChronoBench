import math                                                          # math helpers (ceil, etc.)
import random                                                        # random box placement
import pychrono.core as chrono                                       # Chrono core
import pychrono.vehicle as veh                                       # vehicle module
import pychrono.sensor as sens                                       # sensor module
import pychrono.irrlicht as chronoirr                                # Irrlicht visualization

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn location
init_rot = chrono.QuatFromAngleZ(0)                                  # facing +X
step_size = 2e-3                                                     # integration step (SCM-friendly)

hmmwv = veh.HMMWV_Full()                                             # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                   # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shell
hmmwv.SetChassisFixed(False)                                         # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire — RIGID won't drive on SCM
hmmwv.SetTireStepSize(step_size)                                     # tire substep
hmmwv.Initialize()                                                   # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = hmmwv.GetSystem()                                           # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCM terrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.SCMTerrain(system)                                     # deformable Bekker-Wong soil
terrain.SetSoilParameters(
    2e6,                                                            # Bekker_Kphi
    0,                                                              # Bekker_Kc
    1.1,                                                            # Bekker_n
    0,                                                              # Mohr_cohesion
    30,                                                             # Mohr_friction (deg)
    0.01,                                                           # Janosi_shear (m)
    2e8,                                                            # elastic_K (Pa/m)
    3e4,                                                            # damping_R (Pa.s/m)
)
terrain.AddMovingPatch(                                              # only update cells near the chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                     # local OOBB centre
    chrono.ChVector3d(5, 3, 1),                                     # OOBB dims
)
terrain.SetMeshWireframe(False)                                     # solid terrain mesh
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)  # soil texture
terrain.Initialize(64.0, 64.0, 0.1)                                # length, width, grid resolution (m)

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # tire radius
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # tire width
tire_mat = chrono.ChContactMaterialSMC()                            # tire-soil contact material
tire_mat.SetFriction(0.9)                                          # friction
tire_mat.SetRestitution(0.1)                                       # restitution

TIRE_FAMILY = 1                                                     # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():                          # add collision cylinders so SCM senses the tires
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()                   # the spindle body
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),  # slightly oversized
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),   # axis along wheel
        )
        spindle.EnableCollision(True)                              # enable tire collision
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)                               # tire family
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)                  # tires don't collide with each other
system.GetCollisionSystem().BindAll()                              # rebuild collision models

box_mat = chrono.ChContactMaterialSMC()                            # contact material for the boxes
box_mat.SetFriction(0.8)                                           # box friction
box_mat.SetRestitution(0.01)                                       # box restitution
box_mat.SetYoungModulus(2e7)                                       # box stiffness
random.seed(42)                                                    # deterministic box layout
num_boxes = 8                                                      # number of random boxes
veh_clear = 4.0                                                    # keep boxes this far from the vehicle spawn
for i in range(num_boxes):                                         # introduce randomly positioned boxes
    while True:                                                    # ensure no box spawns on the vehicle
        bx = random.uniform(-25.0, 25.0)                          # random X
        by = random.uniform(-25.0, 25.0)                          # random Y
        if math.hypot(bx - init_loc.x, by - init_loc.y) > veh_clear:  # clear of the vehicle
            break
    box_size = random.uniform(0.4, 0.9)                           # random cube edge
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size, 150, True, True, box_mat)  # visual + collision
    box.SetPos(chrono.ChVector3d(bx, by, box_size / 2 + 0.1))     # rest on the soil surface
    box.SetFixed(False)                                          # dynamic prop
    system.AddBody(box)                                          # add to the world

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV on SCM with boxes and camera")           # window title
vis.SetWindowSize(1280, 720)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera on the chassis
vis.Initialize()                                                   # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # vehicle scenes use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())                              # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                           # interactive driver (truth shape)
render_step_size = 1.0 / 50.0                                      # render cadence (s)
driver.SetSteeringDelta(render_step_size / 1.0)                   # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                  # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                   # braking rate
driver.Initialize()                                               # initialize the driver

manager = sens.ChSensorManager(system)                            # sensor manager on the shared system
intensity = 1.0                                                   # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-20, 20, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(20, -20, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)

offset_pose = chrono.ChFramed(                                     # camera mounted on the chassis
    chrono.ChVector3d(-5.0, 0, 1.5),                             # behind and above the chassis origin
    chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0)),    # slight downward tilt
)
cam = sens.ChCameraSensor(
    hmmwv.GetChassisBody(),                                       # ride on the vehicle chassis
    30,                                                          # update rate (Hz) — physical
    offset_pose,                                                 # offset pose on the chassis
    1280, 720,                                                  # resolution
    1.408,                                                      # horizontal FOV (rad)
)
cam.SetName("Chassis Camera")                                     # sensor name
cam.SetLag(0)                                                    # no lag
cam.SetCollectionWindow(0)                                       # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Chassis Camera"))  # live preview of the camera feed
cam.PushFilter(sens.ChFilterRGBA8Access())                       # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                   # save RGB frames
manager.AddSensor(cam)                                           # register the camera

sim_end = 20.0                                                   # simulation duration (s)
render_steps = math.ceil(render_step_size / step_size)           # physics steps per rendered frame
render_every = max(1, render_steps)                              # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                    # spin to wall-clock
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                            # begin frame
    vis.Render()                                               # draw scene
    vis.EndScene()                                             # end frame
    for _ in range(render_every):
        time = system.GetChTime()                              # current sim time
        driver_inputs = driver.GetInputs()                     # current driver inputs

        driver.Synchronize(time)                               # sync driver
        terrain.Synchronize(time)                              # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)        # sync vehicle (samples SCM)
        vis.Synchronize(time, driver_inputs)                   # sync visualization

        driver.Advance(step_size)                              # advance driver
        terrain.Advance(step_size)                             # advance terrain
        hmmwv.Advance(step_size)                               # advance vehicle (steps the system)
        vis.Advance(step_size)                                 # advance visualization

        manager.Update()                                       # pump the camera sensor each step
        realtime_timer.Spin(step_size)                         # spin in place
        if system.GetChTime() >= sim_end:
            break
