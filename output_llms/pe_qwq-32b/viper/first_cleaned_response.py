import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.0)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(terrain_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
terrain.Initialize()


viper = veh.Viper(vehicle_system=system)
init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT)
viper.SetInitPosition(init_pos)
viper.Initialize()


driver = veh.RoverDriver()  
viper.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.SetCamera(chronoirr.ChVectorD(0, 5, 20))  
vis.SetCameraRotation(chronoirr.ChQuaternionD(1,0,0,0))  
vis.AddTypicalLights()


time_step = 0.005
max_steering = 0.5  
steering_period = 10.0  


while vis.Run():
    current_time = system.GetChTime()
    
    
    if current_time < steering_period:
        steering = max_steering * (current_time / steering_period)
    else:
        steering = max_steering
    
    
    driver.SetSteering(steering)
    driver.SetThrottle(0.5)  
    
    
    system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.Close()