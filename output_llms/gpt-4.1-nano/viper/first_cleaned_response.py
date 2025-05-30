import pychrono.core as chol
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chol.ChSystemNSC()
gravity = chol.ChVectorD(0, -9.81, 0)
system.Set_G_acc(gravity)


ground = chol.ChBodyEasyBox(50, 1, 50, 1000, True, True)
ground.SetPosition(chol.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


groundSurface = chol.ChMaterialSurfaceNSC()
groundSurface.SetFriction(0.8)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(50, 1, 50, chol.ChVectorD(0, 0, 0), chol.QUNIT)
ground.GetCollisionModel().BuildModel()
ground.SetMaterialSurface(groundSurface)



viper = veh.Viper(vehicle_step=1e-3)


initial_pos = chol.ChCoordsysD(chol.ChVectorD(0, 1, 0), chol.Q_from_AngY(0))
viper.GetChassisBody().SetPos(initial_pos)


viper.Initialize(system)


viper.GetVehicle().GetChassis().SetFixed(False)


driver = veh.ChIrrGuiDriver(viper.GetVehicle())


application = irr.ChIrrApp(system, 'Viper Rover Simulation', irr.dimension2du(800, 600))
application.GetSceneManager().AddSkyBox()
application.GetVideoDriver().SetLightPosition(irr.vector3df(30, 45, 30))
application.GetSceneManager().AddCameraSceneNode(irr.E_SCENE_NODE_TYPE.ENU_CAMERA,
                                                 irr.vector3df(0, 5, -15),
                                                 irr.vector3df(0, 2, 0))
application.AssetBindAll()
application.AssetUpdateAll()


application.SetShadows473()
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 1e-3  
simulation_time = 20.0  
steering_change_duration = 10.0  
max_steering_angle = 0.5  


initial_steering = 0.0
target_steering = max_steering_angle


t = 0.0
while t < simulation_time:
    
    if t <= steering_change_duration:
        steering_ratio = t / steering_change_duration
        current_steering = initial_steering + steering_ratio * (target_steering - initial_steering)
    else:
        current_steering = target_steering

    
    driver.SetSteering(current_steering)
    driver.SetThrottle(0.8)  
    driver.SetBraking(0.0)

    
    system.DoStepDynamics(time_step)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    t += time_step

    
    if application.GetDevice().run():
        continue
    else:
        break


application.GetDevice().closeDevice()