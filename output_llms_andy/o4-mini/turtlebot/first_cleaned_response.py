import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono_models.robot.turtlebot as tb_models


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetBodyFixed(True)

ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(chrono.ChVectorD(5, 5, 0.1),
                                  chrono.ChVectorD(0, 0, -0.1))
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)

vis_box = chrono.ChBoxShape()
vis_box.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
vis_box.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, -0.1)
ground.AddVisualShape(vis_box)
system.Add(ground)



bot = tb_models.TurtleBot(system,
                          collision_type=chrono.ChMaterialSurfaceSMC(),
                          fixed=False)


start_pos = chrono.ChVectorD(0, 0, 0.1)
start_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
bot.Initialize(start_pos, start_rot)


app = chronoirr.ChIrrApp(system, "TurtleBot on Rigid Terrain",
                         chronoirr.dimension2du(1024, 768))

app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()

cam_pos = chrono.ChVectorD(1.5, -3, 1.2)
cam_target = chrono.ChVectorD(0, 0, 0.2)
app.AddTypicalCamera(cam_pos, cam_target)

app.AssetBindAll()
app.AssetUpdateAll()


t = 0.0
end_time = 10.0
step_size = 1e-3

while app.Run() and t < end_time:
    app.BeginScene()
    app.DrawAll()

    
    
    
    
    
    
    if 2.0 < t < 4.0:
        left_speed = 1.0
        right_speed = 0.5
    elif 6.0 < t < 8.0:
        left_speed = 0.5
        right_speed = 1.0
    else:
        left_speed = right_speed = 1.0

    
    bot.SetMotorSpeed(left_speed, right_speed)

    
    app.DoStep()
    app.EndScene()
    t += step_size