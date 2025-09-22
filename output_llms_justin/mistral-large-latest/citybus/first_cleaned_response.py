import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
from pychrono.vehicle.wheeled_vehicle import WheeledVehicle
from pychrono.vehicle.CityBus import CityBus
from pychrono.vehicle.terrain import RigidTerrain


chrono.SetChronoDataPath(chrono.GetChronoDataPath() + "/../../../data/")
veh.SetDataPath(chrono.GetChronoDataPath() + "/vehicle/")


system = chrono.ChSystemNSC()
bus = CityBus(system)
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
bus.Initialize(chrono.ChCoordsysD(initLoc, initRot))


bus.GetTire("FL").SetTMeasyTireModel()
bus.GetTire("FR").SetTMeasyTireModel()
bus.GetTire("RL").SetTMeasyTireModel()
bus.GetTire("RR").SetTMeasyTireModel()


terrain = RigidTerrain(system)
terrain.SetTexture("textures/terrain/grass.png")
terrain.SetMeshFile("terrain/meshes/flat_large.obj")


application = chronoirr.ChIrrApp(system, "CityBus Simulation", chronoirr.dimension2du(1280, 720))
application.AddTypicalSky()
application.AddTypicalCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 1))
application.AddTypicalLights()
application.SetTimestep(1.0 / 50.0)  


application.AddCamera(chrono.ChVectorD(0, -6, 2))
application.SetCameraTarget(chrono.ChVectorD(0, 0, 1))


driver = veh.ChDriver(bus)
driver.Initialize()

def driver_inputs(driver):
    
    throttle = 0.0
    steering = 0.0
    braking = 0.0

    
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.KEY_UP):
        throttle = 1.0
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.KEY_DOWN):
        throttle = -1.0
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.KEY_LEFT):
        steering = -1.0
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.KEY_RIGHT):
        steering = 1.0
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.KEY_SPACE):
        braking = 1.0

    driver.SetThrottle(throttle)
    driver.SetSteering(steering)
    driver.SetBraking(braking)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    driver_inputs(driver)
    bus.Synchronize(driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    bus.Advance(application.GetTimestep())
    terrain.Synchronize(application.GetTimestep())
    application.EndScene()

application.Close()