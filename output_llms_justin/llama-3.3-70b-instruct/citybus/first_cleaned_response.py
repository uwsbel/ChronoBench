import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


citybus = veh.CityBus()
citybus.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
citybus.SetChassisFixed(False)
citybus.AddDefaultParts()
citybus.SetSteeringType(veh.SteeringType.RACK_PINION)
citybus.SetTireType(veh.TireType.TME_SIMPLE)
citybus.SetTireModel(veh.TireModel.TME_RIGID)


citybus.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngX(chrono.CH_C_PI_2)))


citybus.GetChassis().GetVisualShape(0).SetVisualizationType(chrono.ChVisualShape.VISUAL_MESH)
citybus.GetChassis().GetCollisionShape(0).SetVisualizationType(chrono.ChVisualShape.VISUAL_PRIMITIVE)


terrain = veh.RigidTerrain(sys)
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"))
terrain.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetPlane(chrono.ChPlane(chrono.ChVectorD(0, 0, 1), 0))


sys.Add(citybus)
sys.Add(terrain)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 1.5))
vis.SetCameraDistance(2.5)
vis.SetCameraAZEL(90, -30, 0)


vis.SetFollowMode(chronoirr.ChVisualSystemIrrlicht.FOLLOW_MODE_CAMERA)


driver = veh InteractiveDriver(citybus)


step_size = 0.02
time_end = 10
fps = 50


while sys.GetChTime() < time_end:
    
    sys.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Synchronize()
    
    
    chrono.ChSleep(1 / fps)


sys.Delete()
vis.Delete()