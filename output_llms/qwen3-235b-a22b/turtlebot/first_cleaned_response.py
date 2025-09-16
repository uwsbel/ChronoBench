import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)


mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.1)


ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(mat, 10, 1, 10)  
ground.GetCollisionModel().BuildModel()


box_vis = chrono.ChBoxShape()
box_vis.GetBoxGeometry().Size = chrono.ChVectorD(10, 1, 10)
ground.AddAsset(box_vis)

system.Add(ground)


turtlebot = robot.Turtlebot(system)
init_pos = chrono.ChVectorD(0, 0.5, 0)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
turtlebot.SetInitPosition(chrono.ChFrameD(init_pos, init_rot))
turtlebot.Initialize()


application = irrlicht.ChIrrApp(system, 'Turtlebot Simulation', irrlicht.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))  
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    
    current_time = system.GetChTime()
    
    
    if current_time < 1.0:
        left_torque, right_torque = 0.0, 0.0  
    elif current_time < 3.0:
        left_torque, right_torque = -1.0, 1.0  
    else:
        left_torque, right_torque = 1.0, -1.0  
    
    
    turtlebot.SetMotorTorqueLeft(left_torque)
    turtlebot.SetMotorTorqueRight(right_torque)
    
    
    system.DoStepDynamics(0.01)
    application.DrawAll()
    application.EndScene()