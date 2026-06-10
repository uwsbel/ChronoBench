import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math






system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)






ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.0)


ground = chrono.ChBodyEasyBox(
    20, 20, 1,        
    1000,             
    True,             
    True,             
    ground_mat        
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)


ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)

system.Add(ground)






viper = robot.Viper(system)


driver = robot.ViperDCMotorControl()
viper.SetDriver(driver)


init_pos = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
viper.Initialize(chrono.ChFramed(init_pos, init_rot))





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper Rover on Rigid Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, 2.5, 1.5),
              chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),   
    chrono.ChVector3d(0, 0, 0.5),        
    3,                                    
    2, 8,                                 
    60                                    
)






time_step = 1e-3
time = 0.0


max_steering = 0.6        
steering_period = 4.0     

print("Starting simulation...")

while vis.Run():
    time = system.GetChTime()

    
    
    if time < steering_period:
        steering = max_steering * (time / steering_period)
    else:
        steering = max_steering * math.sin(
            (time - steering_period) * (math.pi / steering_period)
        )

    
    driver.SetSteering(steering)

    
    viper.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

print("Simulation finished.")