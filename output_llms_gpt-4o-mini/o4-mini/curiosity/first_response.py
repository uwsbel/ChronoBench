import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# --------------------------------------------------------------------------
# 1) INITIALIZE CHRONO ENGINE AND DATA PATH
# --------------------------------------------------------------------------
chrono.SetChronoDataPath("C:/Chrono/data/")     # adjust to your installation
chrono.SetChronoThreads(4)
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# --------------------------------------------------------------------------
# 2) CREATE A RIGID GROUND WITH COLLISIONS AND VISUALIZATION
# --------------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
# collision shape: a large box
ground_collision = chrono.ChBoxShape()
ground_collision.GetBoxGeometry().Size = chrono.ChVectorD(10, 0.1, 10)
ground.AddAsset(ground_collision)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(0.1, 10, 0.1, chrono.ChVectorD(0,0,0))
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)
# visualization: texture
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(chrono.ChTriangleMeshShape())
ground.SetVisualShape(0, chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
ground.AddAsset(vis_mat)

sys.Add(ground)

# --------------------------------------------------------------------------
# 3) CREATE THE CURIOSITY ROVER
# --------------------------------------------------------------------------
# Use the vehicle model provided by Chrono::Vehicle
rover = veh.Curiosity_Rover()
# initialize at (0, 0.5, 0), facing +X
initPos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngY(0))
rover.Initialize(sys, initPos)
# enable collisions and mesh visualization
rover.SetChassisCollision(True)
rover.SetChassisVisualizationType(veh.VisualizationType.MESH)
rover.SetWheelVisualizationType(veh.VisualizationType.MESH)

# --------------------------------------------------------------------------
# 4) SET UP A SIMPLE DRIVER WITH REAL‐TIME STEERING INPUTS
#    (Here we drive steering with a low‐frequency sine wave and constant throttle)
# --------------------------------------------------------------------------
# throttle: constant 30% of max throttle
throttle_fn = chrono.ChFunction_Const(0.3)
# steering: sine wave of amplitude ±0.5 rad, frequency 0.1 Hz
steer_fn = chrono.ChFunction_Sine(0.1, 0.5, 0.0)
# braking: zero
brake_fn = chrono.ChFunction_Const(0.0)

driver = veh.ChDriver(rover)
driver.SetThrottleFunction(throttle_fn)
driver.SetSteeringFunction(steer_fn)
driver.SetBrakingFunction(brake_fn)
driver.Initialize()

# --------------------------------------------------------------------------
# 5) CREATE THE IRRLICHT VISUALIZATION APPLICATION
# --------------------------------------------------------------------------
app = chronoirr.ChIrrApp(
    sys,
    "PyChrono: Curiosity Rover on Rigid Terrain",
    chronoirr.dimension2du(1280, 720)
)
app.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.AddSkyBox()
app.AddTypicalLights()
app.AddShadowAll()
# set camera: position behind and above the rover
cam_pos = chrono.ChVectorD(-4, 2, -4)
cam_target = chrono.ChVectorD(0, 0.5, 0)
app.AddCamera(cam_pos, cam_target)

# bind assets for all bodies (needed before simulation loop)
app.AssetBindAll()
app.AssetUpdateAll()

# --------------------------------------------------------------------------
# 6) SIMULATION LOOP
# --------------------------------------------------------------------------
time_step = 1.0 / 1000  # 1 kHz internally
while app.Run():
    # grab driver inputs (here it's just evaluating our functions)
    driver.GetInputs()
    # render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    # advance dynamics
    sys.DoStepDynamics(time_step)