import os
import math
import numpy as np
from pychrono import chrono as chrono
from pychrono.robot import *
from pychrono.irrlicht import *
from pychrono.vehicle import *


chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


sys = chrono.ChSystemSMC()


class UAZBUS:
    def __init__(self, system, mass=1000, friction=0.2, restitution=0.3, contact_method=chrono.CHContactMethod.HARD):
        self.system = system
        self.mass = mass
        self.friction = friction
        self.restitution = restitution
        self.contact_method = contact_method
        self.gear_ratio = 20  
        self.final_driving_wheels = 4  

    def Initialize(self, position=chrono.ChVector3d(0, 0, 0), rotation=chrono.QuatFromAngleX(chrono.CH_PI)):
        self.system.Add(self)
        self.body = self.system.GetBody()
        self.body.SetMass(self.mass)
        self.body.SetPos(position)
        self.body.SetRotations(rotation)
        self.body.EnableCollision(True)
        
        self.visual_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2, 1.5, 1.5))
        self.body.AddVisualShape(self.visual_shape)
        
        self.wheels = []
        for i in range(self.final_driving_wheels):
            self.wheels.append(self.body.AddWheel(chrono.ChVector3d(0, 0, 0), 
                                                   chrono.ChVector3d(0, 0, 0), 
                                                   self.mass / (self.gear_ratio * 4),
                                                   0.3, 0.2, self.contact_method))
        
        self.gear = self.wheels[0].AddGear(0, self.gear_ratio)

    def SetGear(self, gear, ratio=1):
        self.gear.SetGear(gear, ratio)

    def SetSteeringAngle(self, angle):
        self.wheels[-1].SetSteeringAngle(angle)

    def SetThrottle(self, value):
        self.wheels[-1].SetMotorSpeed(value)


vehicle = UAZBUS(sys, 1000, 0.2, 0.3, chrono.CHContactMethod.HARD)
vehicle.Initialize(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI))
vehicle.gear.SetGear(0, 20)  
vehicle.wheels[0].SetSteeringAngle(0)
vehicle.wheels[-1].SetMotorSpeed(0)  


terrain = vehicle.GetRigidTerrain()
patch = terrain.AddPatch(chrono.ChMaterial(chrono.ChContactMaterialSMC(), 0.5, 0.1, 0.3),
                        chrono.ChVector3d(0, 0, 0),
                        terrain_length=10, terrain_width=10)
terrain.Initialize()
terrain.body.EnableCollision(True)


driver = vehicle.GetDriver()
driver.SetPID(PID=PID(0.1, 0.1, 0.1), sample_time=0.1)
driver.Enable(True)


sys.Add(vehicle)
sys.Add(terrain.terrain_body)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('UAZBUS Simulation')
vis.AddLogo(chrono.GetChronoDataFile('pychrono.png'))
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(0, 5, 10)
cam_dir = chrono.ChVector3d(0, 0, -1)
vis.AddCamera(cam_pos, cam_dir)
vis.AddTypicalLights()


t = 0
while True:
    
    sys.DoStepDynamics(0.001)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if not vis.Run():
        break

    
    t += 0.1
    driver.Update(t)
    vehicle.wheels[-1].SetMotorSpeed(driver.GetThrottle())
    vehicle.wheels[-1].SetSteeringAngle(driver.GetSteering())
    
    
    print(f"Speed: {vehicle.wheels[-1].GetLinearVelocity().x} m/s")


vis.Destroy()
sys.Destroy()