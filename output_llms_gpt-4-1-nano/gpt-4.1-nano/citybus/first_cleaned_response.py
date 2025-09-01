import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath("data/")  


system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, "CityBus Simulation", irr.dimension2du(1024, 768))
application.GetDevice().getTimer().setMaxTimeStep(0.02)
application.AddTypicalLights()
application.AddSignature(irr.vector3df(0, 20, -20))
application.SetCameraAngle(irr.vector3df(15, 15, 15))
application.SetChaseCamera(veh, irr.vector3df(0, 3, -10))
application.AssetBindAll()
application.AssetUpdateAll()


terrain = veh.RigidTerrain(system)


terrainTexture = chrono.AssignImageFile("textures/grass.jpg")
terrain.GetGround()->GetMaterial().SetTexture(terrainTexture)
terrain.Initialize(veh.ChContactMethod_ChContactMethod_NSC)


terrain.SetProjectedBox(chrono.ChCoordsys<>(chrono.V_NULL, chrono.Q_from_AngZ(0)), 100, 1, 100)


bus = veh.CityBus(vehicle_phase=0.0)
bus.SetSystem(system)


initial_pos = chrono.ChVector<>(0, 0.5, 0)
initial_rot = chrono.Q_from_Euler123(0, 0, 0)
bus.GetChassisBody().SetPos(initial_pos)
bus.GetChassisBody().SetRot(initial_rot)



bus.SetChassisVisualizationType(veh.VisualizationType.MESH)

bus.SetWheelVisualizationType(veh.VisualizationType.PRIMITIVE)


for wheel in bus.GetWheels():
    wheel.SetTireType(veh.TireType.PNEUMATIC)
    wheel.SetVisualizationType(veh.VisualizationType.PRIMITIVE)


bus.Initialize(veh.ChVector<>(0, 0.5, 0), initial_rot, 1.0)


driver = veh.ChIrrGuiDriver(application)
driver.Initialize()
driver.BindLightning(0)


bus.GetDriver()->SetUserInput(driver)


application.SetChaseCamera(bus.GetChassisBody(), 10.0, 2.0)


time_step = 1.0 / 50  
application.SetStep(time_step)


while application.GetDevice().run():
    
    application.BeginScene()
    
    driver.Update()
    
    bus.ApplySteering(driver.GetSteering())
    bus.ApplyThrottle(driver.GetThrottle())
    bus.ApplyBraking(driver.GetBraking())

    
    system.DoStepDynamics(time_step)

    
    application.AssetUpdateAll()

    
    application.EndScene()


application.GetDevice().close()