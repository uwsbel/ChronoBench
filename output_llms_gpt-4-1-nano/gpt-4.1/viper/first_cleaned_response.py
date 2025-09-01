import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    length=10.0, width=10.0
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetRestitutionCoefficient(0.01)
patch.SetMaterialSurface(veh.CreateMaterial(veh.ContactMethod_NSC))
patch.SetColor(chrono.ChColor(0.4, 0.6, 0.4))
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 10, 10)
terrain.Initialize()


init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.Q_from_AngZ(0)
viper = veh.Viper(system)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
viper.Initialize()


driver = veh.ViperSpeedDriver()
driver.SetThrottle(0.5)  
driver.SetSteering(0.0)  
viper.SetDriver(driver)


vis = veh.ChWheeledVehicleIrrApp(viper.GetVehicle(), 'PyChrono Viper Rover', irr.dimension2du(1024,768))
vis.AddTypicalLights()
vis.AddTypicalSky()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVectorD(2, 2, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.0), 2.0, 0.5)
vis.SetHUDLocation(500, 20)
vis.AssetBindAll()
vis.AssetUpdateAll()


step_size = 1e-3
end_time = 10.0
steering_time = 3.0     
max_steering = 0.5      


time = 0.0
while vis.Run() and time < end_time:
    
    if time < steering_time:
        steering = max_steering * (time / steering_time)
    else:
        steering = max_steering
    driver.SetSteering(steering)
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    viper.Synchronize(time, driver.GetInputs(), terrain)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    viper.Advance(step_size)
    system.DoStepDynamics(step_size)
    time += step_size

vis.GetDevice().closeDevice()