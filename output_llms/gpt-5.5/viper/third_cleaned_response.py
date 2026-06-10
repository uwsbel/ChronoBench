import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




terrain_length = 20.0
terrain_width = 20.0
terrain_resolution = 0.05



terrain_height = -0.5

terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(
    chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, terrain_height),
        chrono.ChQuaterniond(1, 0, 0, 0)
    )
)
terrain.Initialize(terrain_length, terrain_width, terrain_resolution)





terrain.SetSoilParameters(
    0.2e6,   
    0.0,     
    1.1,     
    0.0,     
    30.0,    
    0.01,    
    4.0e7,   
    3.0e4    
)




rover = viper.Viper(system)

driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 2.5, 1.5),
    chrono.ChVector3d(0, 0, 1)
)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3,
    4,
    10,
    40,
    512
)


time_step = 1e-3




while vis.Run():
    time = system.GetChTime()

    
    steering = 0.0
    driver.SetSteering(steering)

    
    rover.Update()
    terrain.Synchronize(time)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    terrain.Advance(time_step)
    system.DoStepDynamics(time_step)