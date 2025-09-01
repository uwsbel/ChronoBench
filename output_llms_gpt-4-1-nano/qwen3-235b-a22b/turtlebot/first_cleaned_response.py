import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(100, 100, 1, chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().BuildModel()


mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.1)
ground.SetMaterialSurface(mat)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(100, 100, 1)
ground.AddAsset(ground_shape)
ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(ground_texture)


system.Add(ground)


turtlebot = vehicle.ChTurtlebot()
init_pos = chrono.ChVectorD(0, 0, 0.5)  
init_rot = chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 1, 0))  
turtlebot.Initialize(system, chrono.ChCoordsysD(init_pos, init_rot))


application = irr.ChIrrApp(system, 'Turtlebot Simulation', irr.dimension2du(800, 600))
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, -6, 3), chrono.ChVectorD(0, 0, 0))  
application.AddTypicalLights()


application.AssetBindAll()
application.AssetUpdateAll()


step_size = 1e-3  
simulation_time = 0


while application.GetDevice().run():
    
    current_time = system.GetChTime()
    left_torque = 0
    right_torque = 0

    if current_time < 5:
        
        left_torque = 10
        right_torque = 10
    elif current_time < 7:
        
        left_torque = -5
        right_torque = 5
    else:
        
        left_torque = 10
        right_torque = 10

    
    turtlebot.SetLeftMotorTorque(left_torque)
    turtlebot.SetRightMotorTorque(right_torque)

    
    system.DoStepDynamics(step_size)
    simulation_time += step_size

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()