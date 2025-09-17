import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)


viper = veh.Viper()
viper.Initialize(sys)


viper.SetChassisPos(chrono.ChVectorD(0, 0.5, 0))
viper.SetChassisRot(chrono.Q_from_AngX(0))


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(viper)
driver.SetTrack(chrono.ChVectorD(0, 0, 0))
driver.SetCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 1, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(driver.GetCamera())
vis.AddTypicalLights()


time_step = 0.01
steering_time = 10
steering_angle = 0.5


time = 0
while time < 20:
    
    if time < steering_time:
        viper.SetSteering(chrono.ChVectorD(steering_angle * time / steering_time, 0, 0))
    else:
        viper.SetSteering(chrono.ChVectorD(steering_angle, 0, 0))

    
    sys.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    time += time_step

    
    print(f"Time: {time:.2f} s")


vis.GetDevice().run()
vis.GetDevice().drop()