import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh
import math





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))





terrain = chrono.ChBodyEasy()
terrain.SetBodyFixed(True)
terrain.SetMass(1000)
terrain.SetCollide(True)
terrain.SetVisualizationType(chrono.ChVisualizationType.MESH)
terrain.GetCollisionModel().ClearModel()
terrain.GetCollisionModel().AddBox(chrono.ChVectorD(100, 1, 100))
terrain.SetPos(chrono.ChVectorD(0, -0.1, 0))


terrain_texture = chrono.ChTexture()
terrain_texture.SetFilename("terrain.png")  
terrain.SetTexture(terrain_texture)

system.Add(terrain)





vehicle = veh.ChVehicleE90()
vehicle.SetVehicleModel(veh.ChVehicleModel.E90)
vehicle.Initialize(system)
vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetForward(chrono.ChVectorD(1, 0, 0))


vehicle.SetChassisVisualizationType(veh.ChVehicleVisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.ChVehicleVisualizationType.MESH)
vehicle.SetTireVisualizationType(veh.ChVehicleVisualizationType.MESH)


vehicle.GetWheels()[0].GetTire()->SetModel(veh.ChTireModelType.TMEASY)
vehicle.GetWheels()[1].GetTire()->SetModel(veh.ChTireModelType.TMEASY)
vehicle.GetWheels()[2].GetTire()->SetModel(veh.ChTireModelType.TMEASY)
vehicle.GetWheels()[3].GetTire()->SetModel(veh.ChTireModelType.TMEASY)





driver = veh.ChIrrVehicleDriver(vehicle)
driver.SetSteering(0.0)
driver.SetThrottle(0.0)
driver.SetBraking(0.0)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('BMW E90 Sedan Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddSkyBox()


chase_cam = chronoirr.ChChaseCamera()
chase_cam.SetChaseEntity(vehicle.GetVehicle())
chase_cam.SetDistance(3.0)
chase_cam.SetVerticalOffset(1.0)
vis.AddCamera(chase_cam)





system.SetTimestepperType(chrono.ChTimestepper.Type_EulerExplicit)
system.SetStep(0.005)  

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().GetChTime() < 5:
        driver.SetThrottle(0.5)
    else:
        driver.SetThrottle(0.0)
    
    
    system.DoStepDynamics()
    driver.Synchronize(vehicle)