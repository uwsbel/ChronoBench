import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


viper = veh.Viper(system)
viper.SetDriverPos(chrono.ChVectorD(0, 0, 1))
viper.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, -0.2, 0.5), chrono.QUNIT))


driver = veh.ViperDCMotorControl()
driver.SetGains(0.4, 0, 0)
viper.SetDriver(driver)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 0, 5), chrono.ChVectorD(0, 0, 0.5), 10, 2, 10, 40, 512)


step_size = 0.005
t_end = 30
t_steer_start = 5
t_steer_end = 25
steering_max = 0.4  
current_time = 0


while vis.Run() and current_time < t_end:
    
    if t_steer_start <= current_time <= t_steer_end:
        t_normalized = (current_time - t_steer_start) / (t_steer_end - t_steer_start)
        steering = steering_max * chrono.ChSineStep(t_normalized, 0.5)  
        driver.SetSteering(steering)
    
    
    viper.Update()
    terrain.Update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    current_time += step_size