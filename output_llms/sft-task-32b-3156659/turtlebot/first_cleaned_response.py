import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVector3d(0, 0, -0.05))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.ChVisualMaterial(chrono.ChColor(0.8, 0.8, 0.8)))
system.AddBody(ground)

turtlebot = robot.Turtlebot(system, "Turtlebot")
turtlebot.Initialize(chrono.ChFramed(chrono.ChVector3d(0, -2, 0.5), chrono.QUNIT))
turtlebot.SetMotorSpeed(0, 0)  


app = irr.ChIrrApp(system, "Turtlebot Simulation", irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(chrono.ChVector3d(3, 3, 3), chrono.ChVector3d(0, 0, 0.5))
app.AddTypicalLights()
app.SetShowInfos(True)
app.SetContactsDrawMode(1)


time_step = 0.001
sim_time = 0.0
left_speed = 0
right_speed = 0


def update_motor_commands(time):
    global left_speed, right_speed
    if 1.0 < time < 3.0:
        left_speed = 1.0  
        right_speed = 1.0
    elif 3.5 < time < 5.5:
        left_speed = 0.5  
        right_speed = 1.0
    elif 6.0 < time < 8.0:
        left_speed = 1.0  
        right_speed = 0.5
    else:
        left_speed = 0
        right_speed = 0


while app.GetDevice().run():
    update_motor_commands(sim_time)
    turtlebot.SetMotorSpeeds(left_speed, right_speed)
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    system.DoStepDynamics(time_step)
    sim_time += time_step

    
    if sim_time > 0.1:
        system.GetSolver().SetMaxIterations(100)