import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
from pychrono import ChVectorD, ChQuaternionD


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



terrain = veh.RigidTerrain(my_system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize(100, 100)  
my_system.Add(terrain)



viper = veh.Viper(my_system)
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoorD(0, 0.5, 0))  
viper.Initialize()


driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)  
driver.SetThrottleDelta(0.1)  
viper.SetDriver(driver)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0))  
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.AttachSystem(my_system)



time_step = 0.01
sim_time = 10.0
current_time = 0


steering_angle = 0.0
max_steering_angle = 0.5  
steering_rate = 0.1  

while current_time < sim_time:
    
    steering_angle += steering_rate * time_step
    if steering_angle > max_steering_angle:
        steering_angle = max_steering_angle
    if steering_angle < -max_steering_angle:
        steering_angle = -max_steering_angle

    
    driver.SetSteering(steering_angle)
    driver.SetThrottle(0.5)  

    
    viper.Update(time_step)

    
    my_system.DoStepDynamics(time_step)
    current_time += time_step

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    print(f"Simulation time: {current_time:.2f}s")