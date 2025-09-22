import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.robot as viper
from pychrono import irr as chronoirr


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))  
ground.SetFixed(True)  
ground.SetName("ground")  
sys.Add(ground)  


rover = viper.Viper(sys, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0))))


driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)
driver.SetSteeringDelta(0)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1280, 720)  
vis.SetWindowTitle('Viper rover - Rigid terrain')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  
vis.AddTypicalLights()  


time_step = 1e-3


time = 0
while vis.Run():
    time += time_step  
    steering = 0
    max_steering = math.pi / 6  
    
    
    if 2 < time < 7:
        steering = max_steering * (time - 2) / 5  
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5  
    
    driver.SetSteering(steering)  

    
    rover.Update()
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    
    sys.DoStepDynamics(time_step)

    
    time.sleep(time_step)