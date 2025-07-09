import os
import math
import numpy as np
from pychrono import chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *


sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))
sys.SetGroundBody(chrono.ChBody())
sys.GetGroundBody().SetPos(chrono.ChVector3d(0, 0, 0))
sys.GetGroundBody().SetFixed(True)
sys.GetGroundBody().EnableCollision(True)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('TurtleBot Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


robot_body = ChBody()
robot_body.SetMass(30)
robot_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))  
robot_body.SetPos(chrono.ChVector3d(0, 1, 0))  
robot_body.SetVisualShape(ChVisualShapeBox(0.5, 0.5, 0.5))
sys.Add(robot_body)


robot = RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  


terrain_body = ChBody()
terrain_body.SetMass(0)
terrain_body.SetVisualShape(ChVisualShapeBox(100, 0.5, 0.5))
sys.Add(terrain_body)


joint = ChLinkLockLock()
joint.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(joint)


vis.AddCamera(chrono.ChVector3d(0, 5, 10))


motor = ChLinkMotorRotationSpeed()
motor.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(motor)


def left_motor_speed(t):
    return 0.5 * math.sin(t * 2)

def right_motor_speed(t):
    return 0.5 * math.sin(t * 2 + 0.5)


motor.SetSpeedFunction(left_motor_speed)
motor.SetSpeedFunction(right_motor_speed)


time_step = 0.01


while True:
    
    sys.DoStepDynamics(time_step)

    
    motor.SetSpeedFunction(left_motor_speed)
    motor.SetSpeedFunction(right_motor_speed)

    
    if robot_body.GetContactCount() > 0:
        
        normal_force = robot_body.GetContactForce().GetNormal()
        robot_body.SetLinearDamping(normal_force * 0.1)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if not vis.Run():
        break

import os
import math
import numpy as np
from pychrono import chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *


sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))
sys.SetGroundBody(chrono.ChBody())
sys.GetGroundBody().SetPos(chrono.ChVector3d(0, 0, 0))
sys.GetGroundBody().SetFixed(True)
sys.GetGroundBody().EnableCollision(True)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('TurtleBot Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


robot_body = ChBody()
robot_body.SetMass(30)
robot_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))  
robot_body.SetPos(chrono.ChVector3d(0, 1, 0))  
robot_body.SetVisualShape(ChVisualShapeBox(0.5, 0.5, 0.5))
sys.Add(robot_body)


robot = RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))  


terrain_body = ChBody()
terrain_body.SetMass(0)
terrain_body.SetVisualShape(ChVisualShapeBox(100, 0.5, 0.5))
sys.Add(terrain_body)


joint = ChLinkLockLock()
joint.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(joint)


vis.AddCamera(chrono.ChVector3d(0, 5, 10))


motor = ChLinkMotorRotationSpeed()
motor.Initialize(robot_body, terrain_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(motor)


def left_motor_speed(t):
    return 0.5 * math.sin(t * 2)

def right_motor_speed(t):
    return 0.5 * math.sin(t * 2 + 0.5)


motor.SetSpeedFunction(left_motor_speed)
motor.SetSpeedFunction(right_motor_speed)


time_step = 0.01


while True:
    
    sys.DoStepDynamics(time_step)

    
    motor.SetSpeedFunction(left_motor_speed)
    motor.SetSpeedFunction(right_motor_speed)

    
    if robot_body.GetContactCount() > 0:
        
        normal_force = robot_body.GetContactForce().GetNormal()
        robot_body.SetLinearDamping(normal_force * 0.1)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if not vis.Run():
        break