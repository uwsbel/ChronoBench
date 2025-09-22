import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
from pychrono.vehicle import Viper
from pychrono import ChVectorD


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



terrain = veh.Terrain(system)
terrain.SetContactFrictionCoefficient(0.8)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)


patch = terrain.AddPatch(chrono.GetChronoDataFile("terrain/flat.pov"),
                         chrono.ChVectorD(0, 0, 0),
                         chrono.ChVectorD(0, 0, 0),
                         100, 100)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))



viper = Viper()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoorD(0, 0.5, 0))
viper.SetInitFwdVel(1)
viper.Initialize(system, terrain)


driver = veh.ChDriverVI()
driver.Initialize(viper.GetVehicle())


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0), chrono.ChVectorD(0, 0.5, 0))
vis.AddTypicalLights()
vis.AssetBindAll()
vis.AssetUpdateAll()


time_step = 0.01
max_time = 10.0
steering_time = 5.0  

current_time = 0
while current_time < max_time:
    
    current_time = system.GetChTime()

    
    if current_time < steering_time:
        steering_angle = 0.5 * (1 - current_time/steering_time)  
    else:
        steering_angle = 0

    
    driver.SetSteering(steering_angle)
    driver.SetThrottle(0.5)  

    
    driver.Synchronize(current_time)
    viper.Synchronize(current_time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    if current_time > 0.5:
        print(f"Time: {current_time:.2f}, Steering: {steering_angle:.2f}, Position: {viper.GetChassisBody().GetPos().x:.2f}")


vis.CloseDevice()