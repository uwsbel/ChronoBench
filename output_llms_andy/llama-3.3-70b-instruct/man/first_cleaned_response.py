import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


truck = veh.VehicleMAN10t()
truck.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
truck.SetChassisCollisionBox(chrono.ChBox(chrono.ChVectorD(2, 1, 1), chrono.ChVectorD(1, 1, 1)))
truck.SetChassisVisualization(chrono.ChVisualization(chrono.ChVisualMaterial(), chrono.ChVisualBox(chrono.ChVectorD(2, 1, 1), chrono.ChVectorD(1, 1, 1))))


sys.Add(truck)


terrain = chrono.ChTerrain()
terrain.SetPlane(chrono.ChVectorD(0, 1, 0), 0)
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
sys.Add(terrain)


tire_model = veh.TMEASY()
truck.SetTireModel(tire_model)


truck.SetSteering(0)
truck.SetThrottle(0)
truck.SetBraking(0)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
vis.SetLogo(chrono.ChVectorD(0.5, 0.9))
vis.SetTypicalLighting()


chase_cam = vis.AddCamera()
chase_cam.SetFollowNode(truck.GetChassisBody())
chase_cam.SetDistance(5)
chase_cam.SetElevation(10)


driver = veh.DriverRealtime()
driver.SetVehicle(truck)
driver.SetSteeringGain(1)
driver.SetThrottleGain(1)
driver.SetBrakingGain(1)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()