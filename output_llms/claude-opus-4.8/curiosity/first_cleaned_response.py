import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())


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






rover = robot.Curiosity(system)


driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)


init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)


rover.Initialize(chrono.ChFramed(init_pos, init_rot))





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover on Rigid Terrain')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


vis.AddCamera(chrono.ChVector3d(3, 3, 1.5),   
              chrono.ChVector3d(0, 0, 0))      


vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(5, 5, 10),    
    chrono.ChVector3d(0, 0, 0),     
    20,                             
    1, 15,                          
    55,                             
    512                             
)
vis.EnableShadows()





time_step = 1e-3
time = 0.0

while vis.Run():
    time = system.GetChTime()

    
    
    steering = 0.5 * math.sin(time * 0.5)
    driver.SetSteering(steering)

    
    rover.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

print("Simulation finished.")