import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
from math import pi






time_step = 0.01
end_time = 30.0


terrain_length = 20.0
terrain_width = 20.0
terrain_height = 0.5


rover_length = 1.0
rover_width = 0.8
rover_height = 0.5
wheel_radius = 0.2
wheel_width = 0.1






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetMaxItersSolverSpeed(50)
system.SetMaxItersSolverStab(50)
system.SetTolForce(1e-5)






terrain = chrono.ChBodyEasyBox(terrain_length, terrain_height, terrain_width, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -terrain_height/2, 0))
terrain.SetBodyFixed(True)
terrain.SetCollide(True)
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurfaceNSC().SetFriction(0.8)
system.Add(terrain)






rover_chassis = chrono.ChBodyEasyBox(rover_length, rover_height, rover_width, 50, True, True)
rover_chassis.SetPos(chrono.ChVectorD(0, rover_height/2 + wheel_radius, 0))
rover_chassis.SetCollide(True)
rover_chassis.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
rover_chassis.GetMaterialSurfaceNSC().SetFriction(0.5)
system.Add(rover_chassis)






wheel_positions = [
    chrono.ChVectorD(rover_length/2 - wheel_width/2, 0, rover_width/2 - wheel_width/2),  
    chrono.ChVectorD(rover_length/2 - wheel_width/2, 0, -rover_width/2 + wheel_width/2), 
    chrono.ChVectorD(-rover_length/2 + wheel_width/2, 0, rover_width/2 - wheel_width/2), 
    chrono.ChVectorD(-rover_length/2 + wheel_width/2, 0, -rover_width/2 + wheel_width/2)  
]

wheels = []
wheel_motors = []

for i, pos in enumerate(wheel_positions):
    
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 1000, True, True)
    wheel.SetPos(rover_chassis.GetPos() + pos)
    wheel.SetCollide(True)
    wheel.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    wheel.GetMaterialSurfaceNSC().SetFriction(0.8)
    system.Add(wheel)
    wheels.append(wheel)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(rover_chassis, wheel, chrono.ChFrameD(pos, chrono.QUNIT))
    motor.SetSpeedFunction(chrono.ChFunction_Const(0))  
    system.Add(motor)
    wheel_motors.append(motor)

    
    suspension = chrono.ChLinkLockRevolute()
    suspension.Initialize(rover_chassis, wheel, chrono.ChFrameD(pos, chrono.QUNIT))
    system.Add(suspension)





class RoverMotorDriver(chrono.ChReportContactCallback):
    def __init__(self, wheel_motors):
        super().__init__()
        self.wheel_motors = wheel_motors
        self.steering_angle = 0.0
        self.throttle = 0.0

    def SetSteering(self, angle):
        self.steering_angle = angle

    def SetThrottle(self, throttle):
        self.throttle = throttle

    def OnReportContact(self, contact_info):
        
        left_speed = self.throttle * (1 - self.steering_angle)
        right_speed = self.throttle * (1 + self.steering_angle)

        
        for i in range(4):
            if i < 2:  
                self.wheel_motors[i].SetSpeedFunction(chrono.ChFunction_Const(right_speed))
            else:      
                self.wheel_motors[i].SetSpeedFunction(chrono.ChFunction_Const(left_speed))


motor_driver = RoverMotorDriver(wheel_motors)
system.GetContactContainer().ReportAllContacts(motor_driver)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 3, 0), chrono.ChVectorD(0, 0, 0))
vis.AddLightWithShadow(chrono.ChVectorD(5, 10, 5), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512)
vis.SetShadows(True)


vis.AttachSystem(system)


terrain_vis = chrono.ChObjShapeFile()
terrain_vis.SetFilename(chrono.GetChronoDataFile("textures/terrain.jpg"))
terrain.AddVisualShape(terrain_vis)


rover_vis = chrono.ChObjShapeFile()
rover_vis.SetFilename(chrono.GetChronoDataFile("textures/metal.jpg"))
rover_chassis.AddVisualShape(rover_vis)

for wheel in wheels:
    wheel_vis = chrono.ChObjShapeFile()
    wheel_vis.SetFilename(chrono.GetChronoDataFile("textures/rubber.jpg"))
    wheel.AddVisualShape(wheel_vis)






vis.SetUserInteractionCallback(motor_driver)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    if system.GetChTime() < 5:
        motor_driver.SetThrottle(2.0)
        motor_driver.SetSteering(0.0)
    elif system.GetChTime() < 10:
        motor_driver.SetThrottle(1.5)
        motor_driver.SetSteering(0.5)
    elif system.GetChTime() < 15:
        motor_driver.SetThrottle(1.5)
        motor_driver.SetSteering(-0.5)
    else:
        motor_driver.SetThrottle(0.0)
        motor_driver.SetSteering(0.0)

    
    if system.GetChTime() > end_time:
        break





vis.Close()