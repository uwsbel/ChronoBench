import os
import math
import random
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # core data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # vehicle data path

initLoc = chrono.ChVector3d(0, 0, 0.5)                                 # HMMWV spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT, no rotation

step_size = 1e-3                                                       # integration step
tire_step_size = step_size                                            # tire substep matches step

hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                     # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                  # chassis has no collision shape
hmmwv.SetChassisFixed(False)                                           # chassis is free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))           # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire for SCM driving
hmmwv.SetTireStepSize(tire_step_size)                                 # tire integration step
hmmwv.Initialize()                                                     # build the vehicle

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # BULLET before SCM
system = hmmwv.GetSystem()                                             # the wrapper-owned ChSystem

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                 # truth vehicle banner

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)         # mesh visualization, all parts
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.SCMTerrain(system)                                       # deformable SCM terrain
terrain.SetSoilParameters(                                            # custom Bekker-Wong soil params
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — sinkage exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear modulus (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)            # false-color sinkage plot
terrain.AddMovingPatch(                                                # moving patch follows chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                       # local OOBB centre
    chrono.ChVector3d(5, 3, 1),                                       # OOBB dimensions (m)
)
terrain.SetMeshWireframe(False)                                        # solid mesh, not wireframe
terrain.Initialize(20.0, 20.0, 0.02)                                  # length, width, grid resolution (m)

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # tire radius
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # tire width
tire_mat = chrono.ChContactMaterialSMC()                               # tire-soil contact material
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
TIRE_FAMILY = 1                                                        # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():                            # explicit collision cylinders for TMEASY
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
system.GetCollisionSystem().BindAll()                                  # rebuild collision models

box_mat = chrono.ChContactMaterialSMC()                                # contact material for boxes
box_mat.SetFriction(0.9)
box_mat.SetRestitution(0.01)
box_mat.SetYoungModulus(2e7)
support = chrono.ChBodyEasyBox(20.0, 20.0, 0.2, 1000, False, True, box_mat)  # hidden rigid support plane
support.SetPos(chrono.ChVector3d(0, 0, -0.1))                          # top at z=0 (SCM rest plane)
support.SetFixed(True)
support.EnableCollision(True)
system.AddBody(support)

random.seed(0)                                                         # reproducible box layout
num_boxes = 8                                                          # number of random props
for ib in range(num_boxes):
    bx = random.uniform(-8.0, 8.0)                                     # random x
    by = random.uniform(-8.0, 8.0)                                     # random y
    if abs(bx - initLoc.x) < 3.0 and abs(by - initLoc.y) < 3.0:        # don't spawn inside the vehicle
        continue
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, box_mat)  # a random prop box
    box.SetPos(chrono.ChVector3d(bx, by, 0.25))                        # rest on the support plane
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))     # reddish box
    system.AddBody(box)

manager = sens.ChSensorManager(system)                                 # sensor manager
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 5000.0)   # point lights
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(1, 1, 1), 5000.0)
manager.scene.AddPointLight(chrono.ChVector3f(-9, 2.5, 100), chrono.ChColor(1, 1, 1), 5000.0)

offset_pose = chrono.ChFramed(                                         # camera offset on the chassis
    chrono.ChVector3d(-8, 0, 3),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(                                             # chassis-mounted camera sensor
    hmmwv.GetChassisBody(),                                           # ride on the chassis
    30,                                                               # update_rate (Hz)
    offset_pose,                                                      # offset pose on the body
    1280, 720,                                                        # resolution
    1.408,                                                            # horizontal FOV (rad)
)
cam.SetName("Chassis Camera")
cam.SetLag(0)                                                          # no lag
cam.SetCollectionWindow(0)                                            # zero exposure window
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "HMMWV Camera"))    # visualize the camera feed
cam.PushFilter(sens.ChFilterRGBA8Access())                            # host RGBA8 access
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                         # save RGB frames
manager.AddSensor(cam)                                                # register the camera

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle Irrlicht visual system
vis.SetWindowTitle('HMMWV Demo')                                       # window title
vis.SetWindowSize(1280, 1024)                                          # window pixels
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)           # chase camera trackpoint/dist/height
vis.Initialize()                                                       # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # logo (after Initialize)
vis.AddSkyBox()                                                        # sky box
vis.AddLightDirectional()                                              # directional light (vehicle default)
vis.AttachVehicle(hmmwv.GetVehicle())                                  # bind vehicle for chase camera

driver = veh.ChInteractiveDriverIRR(vis)                               # interactive keyboard driver
render_step_size = 1.0 / 50.0                                          # render at 50 fps
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking rate
driver.Initialize()                                                    # build the driver

render_every = max(1, round(render_step_size / step_size))            # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                         # wall-clock pacing
step_number = 0                                                        # physics step counter
while vis.Run():                                                       # plain real-time loop
    time = system.GetChTime()                                         # current sim time

    if step_number % render_every == 0:                               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # current driver inputs

    driver.Synchronize(time)                                         # synchronize subsystems
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)                                        # advance subsystems
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                          # advances the wrapper system
    vis.Advance(step_size)

    manager.Update()                                                 # pump the camera sensor once per step

    step_number += 1                                                 # next step
    realtime_timer.Spin(step_size)                                   # spin to wall-clock
