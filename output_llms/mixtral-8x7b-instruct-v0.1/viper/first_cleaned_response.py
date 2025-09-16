import chrono
import irrlicht


chrono.ChronoPy_SetFlag_ChronoOutputFile(False)
my_chrono = chrono.ChronoEngine()


my_chrono.GetSystem().Add(chrono.RigidTerrain())
my_chrono.GetSystem().Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = my_chrono.Add(chrono.ViperChassis())
driver = my_chrono.Add(chrono.ViperDriver(vehicle))


my_vis = my_chrono.GetSystem().Add(irrlicht.ChIrrApp(my_chrono.GetSystem(), "Viper on Terrain"))
my_vis.SetChaseCamera(chrono.ChVectorD(10, 5, 5), chrono.ChVectorD(0, 0, 0))
my_vis.AddLight(chrono.ChVectorD(0, 10, 0), 0.4, 0.4, 0.4)
my_vis.AddLight(chrono.ChVectorD(0, -10, 0), 0.4, 0.4, 0.4)


step = 0
steer_time = 5.0  
steer_change = 0.1  
current_steer = 0
while step < 100:
    driver.SetSteer(current_steer)
    my_chrono.DoStepDynamics()
    my_vis.UpdateVideo()
    my_vis.Render()
    step += 1

    if step <= steer_time:
        current_steer += steer_change
    else:
        current_steer -= steer_change


my_vis.Run()