import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


truck = veh.MAN_10t(system)
truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
truck.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
truck.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC)
truck.Initialize()


for axle in truck.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.ChTMEASYTire("TMEASY_Tire")
        tire.Initialize(wheel, veh.WheelID())
        tire.SetRadius(0.5)
        tire.SetWidth(0.2)
        tire.SetTMEASYParams()  
        wheel.SetTire(tire)


terrain = chrono.ChBody()
terrain.SetBodyFixed(True)
terrain.SetCollide(True)
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetCollisionModel().AddBox(100, 1, 100)  
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain_vis = chrono.ChBoxShape()
terrain_vis.GetBoxGeometry().Size = chrono.ChVectorD(50, 0.5, 50)  
terrain_vis.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.AddAsset(terrain_vis)
system.Add(terrain)


application = irrlicht.ChIrrApp(system, 'MAN 10t Truck Simulation', irrlicht.dimension2du(1280, 720))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))  
application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(application.GetDevice(), truck.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


while application.GetDevice().run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    truck.Synchronize(time, driver.GetInputs())
    
    
    system.DoStepDynamics(chrono.ChTimeStep(1e-3))
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()