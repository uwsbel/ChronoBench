import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  





body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetMass(1000)
body_ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
body_ground.SetShapeBoxes(0.5, 0.1, 10)
body_ground.GetCollisionModel().ClearModel()
body_ground.GetCollisionModel().AddBox(0.5, 0.1, 10)
body_ground.GetCollisionModel().BuildModel()
system.Add(body_ground)






rover = veh.ChRover()
rover.SetVehicleModelPath("path/to/your/rover_model.py")  
rover.Initialize(system)
rover.SetChassisHeight(0.3)
rover.SetWheelRadius(0.1)
rover.SetWheelMass(1.0)
rover.SetEngineForceMax(500)
rover.SetBrakeForceMax(200)
rover.SetSteeringLimit(0.5) 


rover.SetPos(chrono.ChVectorD(0, 0.5, 0))


system.Add(rover)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))
vis.AddTypicalLights()


vis.SetShadows(True)








time_step = 0.005
time_end = 10

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    steering_input = 0.0  
    if vis.GetSystem().GetChTime() < 5:
        steering_input = 0.2  
    else:
        steering_input = -0.2 

    
    rover.SetSteeringAngle(steering_input)

    
    system.DoStepDynamics(time_step)