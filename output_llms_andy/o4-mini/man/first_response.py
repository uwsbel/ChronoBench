#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.man as man
import pychrono.vehicle.tire as veh_tire
import pychrono.vehicle.driver as veh_driver
import pychrono.irrlicht as irr

# ------------------------------------------------------------------------
# 1) INITIALIZE THE CHRONO SYSTEM
# ------------------------------------------------------------------------
chrono.SetChronoDataPath("/path/to/chrono/data/")   # point this to your Chrono::Data folder
system = chrono.ChSystemSMC()

# ------------------------------------------------------------------------
# 2) CREATE A RIGID TERRAIN
# ------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
ground_patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                      chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))),
    200, 200)
ground_patch.SetColor(veh.ChColor(0.8, 0.8, 0.5))
ground_patch.SetTexture(
    veh.GetDataFile("terrain/textures/grass.jpg"), 50, 50)
ground_patch.SetLogo(
    veh.GetDataFile("terrain/textures/gearlogo_pychrono_alpha.png"), 0.25)
terrain.Initialize()

# ------------------------------------------------------------------------
# 3) CREATE THE MAN 10t TRUCK
# ------------------------------------------------------------------------
vehicle = man.MAN_10t(veh.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_BOX)
# initial position: x=0,y=0,z=0.6 with zero yaw
vehicle.SetInitPosition(chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0.6),
    chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
# use mesh-based visualization
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()

# ------------------------------------------------------------------------
# 4) CREATE TMEASY TIRES AND ATTACH THEM
# ------------------------------------------------------------------------
tire_json = veh.GetDataFile("vehicle/tire/TMeasy/TMeasy.json")
tires = []
for axle in range(vehicle.GetNumberAxles()):
    for side in [0, 1]:   # 0=left, 1=right
        t = veh_tire.TMeasyTire()
        t.SetTireDataFile(tire_json)
        t.Initialize(vehicle.GetWheelBody(axle, side),
                     vehicle.GetWheelFrame(axle, side))
        tires.append(t)

# ------------------------------------------------------------------------
# 5) SET UP THE IRRLICHT VISUALIZER
# ------------------------------------------------------------------------
vis = veh.ChVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono: MAN 10t Truck on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.Initialize()
# skybox
vis.AddSkyBox(
    veh.GetDataFile("skybox/skybox_px.jpg"),
    veh.GetDataFile("skybox/skybox_nx.jpg"),
    veh.GetDataFile("skybox/skybox_py.jpg"),
    veh.GetDataFile("skybox/skybox_ny.jpg"),
    veh.GetDataFile("skybox/skybox_pz.jpg"),
    veh.GetDataFile("skybox/skybox_nz.jpg"))
# logo in the corner
vis.AddLogo(veh.GetDataFile("logo_pychrono_alpha.png"))
# directional lights
vis.AddLightDirectional(chrono.ChVectorD(-1,  1, -1), 1.0)
vis.AddLightDirectional(chrono.ChVectorD( 1, -1, -0.5), 0.7)
# chase camera behind the truck
vis.AddCamera(chrono.ChVectorD(-6, -6, 3),
              chrono.ChVectorD(0,  0, 1))

# ------------------------------------------------------------------------
# 6) CREATE A REAL‐TIME KEYBOARD DRIVER
# ------------------------------------------------------------------------
driver = veh_driver.ChIrrGuiDriver(vis)
driver.SetInputMode(veh_driver.InputMode_KEYBOARD)
driver.Initialize()

# ------------------------------------------------------------------------
# 7) RUN THE SIMULATION
# ------------------------------------------------------------------------
step_size = 1e-3
render_step = 1.0 / 60.0
next_render_time = 0.0

while vis.Run():
    t = system.GetChTime()

    # 7a) COLLECT DRIVER INPUTS
    driver.Synchronize(t)
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking = driver.GetBraking()

    # 7b) SYNCHRONIZE MODULES
    terrain.Synchronize(t)
    vehicle.Synchronize(t, steering, throttle, braking, terrain)
    for tr in tires:
        tr.Synchronize(t)
    vis.Synchronize(t, vehicle.GetChassisBody(),
                    vehicle.GetShaftsColor())

    # 7c) RENDER (at fixed rate)
    if t >= next_render_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        next_render_time += render_step

    # 7d) ADVANCE ALL MODULES
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    for tr in tires:
        tr.Advance(step_size)
    vis.Advance(step_size)
    system.DoStepDynamics(step_size)