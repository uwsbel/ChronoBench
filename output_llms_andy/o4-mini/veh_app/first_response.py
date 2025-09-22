import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.gator as gator
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# -----------------------------------------------------------------------------
# 1. Initialize Chrono system
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath('./')   # set this to your Chrono data folder if needed
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# -----------------------------------------------------------------------------
# 2. Create the Gator vehicle
# -----------------------------------------------------------------------------
veh.SetChronoDataPath(chrono.GetChronoDataPath())
my_gator = gator.Gator()
my_gator.SetContactMethod(veh.ChContactMethod_SMC)
my_gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0.2), chrono.QUNIT))
my_gator.SetTireType(veh.TireModelType_PAC89)
my_gator.SetDrivelineType(veh.DrivelineType_SPKV)
my_gator.SetBrakeType(veh.BrakeType_SHAFTS)
my_gator.Initialize()

# Set visualization types for various subsystems
my_gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
my_gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
my_gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
my_gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
my_gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# -----------------------------------------------------------------------------
# 3. Create a rigid terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.CSYSNORM,                # plane at z=0
                         chrono.ChVectorD(0, 0, 0),
                         200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
terrain.Initialize()

# -----------------------------------------------------------------------------
# 4. Create the Irrlicht visualization & interactive driver
# -----------------------------------------------------------------------------
app = chronoirr.ChIrrApp(my_gator.GetVehicle(), system, 'Gator Demo', chronoirr.dimension2du(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0.0, -6.0, 1.7), chronoirr.vector3df(0, 0, 0.5))
app.AssetBindAll()
app.AssetUpdateAll()

driver = veh.ChIrrGuiDriver(app)
driver.Initialize()

# -----------------------------------------------------------------------------
# 5. Create the sensor manager and attach lights + camera to the chassis
# -----------------------------------------------------------------------------
sens.SetChronoDataPath(chrono.GetChronoDataPath() + 'sensor/')

sensor_manager = sens.ChSensorManager(system)

# 5a) Two point lights (one above & in front, one to the side)
pose_light1 = chrono.ChFrameD(chrono.ChVectorD(1.0, 0.0, 1.5), chrono.QUNIT)
light1 = sens.ChPointLightSensor(parent = my_gator.GetChassisBody(),
                                 updateRate = 30,
                                 offsetPose = pose_light1,
                                 dims = sens.ChVector2<int>(1,1))   # dummy dims
light1.SetLightProperties(color = sens.ChColor(1,1,1), intensity = 3000)
sensor_manager.AddSensor(light1)

pose_light2 = chrono.ChFrameD(chrono.ChVectorD(0.0, 1.0, 1.2), chrono.QUNIT)
light2 = sens.ChPointLightSensor(parent = my_gator.GetChassisBody(),
                                 updateRate = 30,
                                 offsetPose = pose_light2,
                                 dims = sens.ChVector2<int>(1,1))
light2.SetLightProperties(color = sens.ChColor(1,1,0.8), intensity = 1500)
sensor_manager.AddSensor(light2)

# 5b) A color camera facing forward from the chassis
cam_pose = chrono.ChFrameD(chrono.ChVectorD(0.6, 0, 1.2), 
                           chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0,1,0)))
camera = sens.ChCameraSensor(parent       = my_gator.GetChassisBody(),
                             updateRate   = 30,
                             offsetPose   = cam_pose,
                             imageWidth   = 640,
                             imageHeight  = 480,
                             fov_y        = 1.0472)  # 60 degrees
camera.SetName("FrontCam")
camera.PushFilter(sens.ChFilterVisualize(640, 480))   # pop up an OpenGL window
sensor_manager.AddSensor(camera)

# Final initialization of sensors
sensor_manager.InitializeAll()

# -----------------------------------------------------------------------------
# 6. Simulation loop
# -----------------------------------------------------------------------------
timestep = 1.0/60
while app.GetDevice().run():
    # Irrlicht draw call
    app.BeginScene()
    app.DrawAll()

    # 1) Driver inputs
    driver.Synchronize(my_gator.GetChassis()->GetPos(), my_gator.GetVehicleSpeed(), 0)

    # 2) Terrain does not need explicit sync
    #    but for consistency:
    terrain.Synchronize(0,0,0)

    # 3) Vehicle modules
    my_gator.Synchronize(driver.GetInputs(), 
                         terrain.GetNormal(), 
                         terrain.GetContactForces())
    my_gator.Advance(timestep)

    # 4) Sensors
    sensor_manager.UpdateAll()

    app.EndScene()

    # advance time in mechanics
    system.DoStepDynamics(timestep)