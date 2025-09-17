import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.self_collision as sc


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


collision_system = chrono.ChCollisionSystemNSC()
system.Add(collision_system)


terrain = chrono.ChTerrain()
terrain.Create(chrono.ChVectorD(100, 100), 10, 10)  
terrain.SetTexture(chrono.ChTexture("textures/terrain.jpg"))  
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterial().SetFriction(0.9)
system.Add(terrain)


vehicle = veh.ChVehicleE90()
vehicle.SetVehicleModel("e90")
vehicle.SetEngineType(veh.ChEngineModelType.AUTOMATIC)
vehicle.SetTransmissionModel(veh.ChTransmissionModelType.SIMPLE)
vehicle.SetDifferentialModel(veh.ChDifferentialModelType.SIMPLE)
vehicle.SetBrakeModel(veh.ChBrakeModelType.HYDRAULIC)
vehicle.SetSteeringModel(veh.ChSteeringModelType.PITMAN)
vehicle.SetTireModel(veh.ChTireModelType.TMEASY)
vehicle.SetChassisHeight(0.4)
vehicle.SetInitialPosition(chrono.ChVectorD(0, 1, 0))
vehicle.SetInitialRotation(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(vehicle)


for i in range(4):
    tire = vehicle.GetTire(i)
    tire.SetTireModel(veh.ChTireModelType.TMEASY)
    tire.SetRadius(0.33)
    tire.SetWidth(0.25)
    tire.SetPressure(250000)


driver = veh.ChInteractiveDriver(vehicle)
driver.SetSteeringDelta(10.0)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
driver.SetHandbrakeDelta(0.1)
driver.SetMaxSteeringAngle(0.5)  
system.Add(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('BMW E90 Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -15))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo()
vis.AddTerrain(terrain)


camera = vis.GetCamera(0)
camera.SetChaseCamera(True)
camera.SetChaseDistance(5.0)
camera.SetChaseVerticalOffset(2.0)
camera.SetChaseHorizontalOffset(0.0)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)