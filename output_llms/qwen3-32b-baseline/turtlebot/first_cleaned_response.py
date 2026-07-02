import pychrono as chrono
from pychrono import chronoirr, robot


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


turtlebot = robot.Turtlebot(system, "Turtlebot")
chassis = turtlebot.GetChassisBody()
chassis.SetPos(chrono.ChVectorD(0, 0, 0.5))  
chassis.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  


application = chronoirr.ChIrrApp(system, 'Turtlebot Simulation', chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.SetCamera(chronoirr.vector3df(0, 3, 5), chronoirr.vector3df(0, 0, 0))
application.SetAmbientLight(chronoirr.SColorf(1, 1, 1, 1))
application.AddLightWithShadow(chronoirr.vector3df(10, 10, 10), chronoirr.vector3df(0, 0, 0), 2, 25, 100, 50, 500)
application.Build()


time_step = 0.001
sim_time = 0.0


def set_motor_speeds(time):
    if time < 2.0:
        left = 100  
        right = 100
    elif time < 4.0:
        left = 50   
        right = 100
    else:
        left = 100  
        right = 50
    turtlebot.SetMotorSpeed(left, right)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    set_motor_speeds(sim_time)
    
    
    system.DoStepDynamics(time_step)
    sim_time += time_step
    
    application.EndScene()