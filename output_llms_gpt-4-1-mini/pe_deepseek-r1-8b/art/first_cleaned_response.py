import os
import math
import numpy as np
import pychrono
from pychrono import chrono, irrrlicht, robot, vehicle
from pychrono.robot import ChRobot
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere
from pychrono.vehicle import ARTcar


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemNSC()


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ARTcar Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  


car = ARTcar(sys)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
car.SetContactMethod(chrono.ChContactMethodType.Differentiable_Slide)
car.SetChassisCollisionType(chrono.ChCollisionType.Rigid)
car.Initialize()


terrain = vehicle.RigidTerrain(sys)
terrain.Initialize()
patch = terrain.AddPatch(
    material=chrono.ChMaterial(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    lengthX=100,
    lengthY=100,
    texturePath=os.path.join(os.path.dirname(__file__), "asphalt.dds")
)
terrain.SetPatchHeight(0)


class Driver:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.steering = 0
        self.throttle = 0
        self.braking = 0

    def Control(self, delta_time):
        self.vehicle.SetSteering(self.steering)
        self.vehicle.SetThrottle(self.throttle)
        self.vehicle.SetBraking(self.braking)

driver = Driver(car)
driver.EnableDriver(True)



chassis = car.AddChassis()
chassis.SetName("Chassis")
chassis.SetMass(1000)
chassis.SetInertiaXX(chrono.ChVector3d(5, 5, 5))
chassis.SetPos(chrono.ChVector3d(0, 0, 0))
chassis.EnableCollision(True)


wheel_fl = car.AddWheels("Front Left Wheel")
wheel_fl.SetName("Front Left Wheel")
wheel_fl.SetRadius(0.3)
wheel_fl.SetMass(10)
wheel_fl.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
wheel_fl.SetPos(chrono.ChVector3d(0.5, 0, 0))
wheel_fl.SetRot(chrono.QuatFromAxisAngle(chrono.ChVector3d(1, 0, 0), math.radians(90)))


wheel_fr = car.AddWheels("Front Right Wheel")
wheel_fr.SetName("Front Right Wheel")
wheel_fr.SetRadius(0.3)
wheel_fr.SetMass(10)
wheel_fr.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
wheel_fr.SetPos(chrono.ChVector3d(-0.5, 0, 0))
wheel_fr.SetRot(chrono.QuatFromAxisAngle(chrono.ChVector3d(1, 0, 0), math.radians(-90)))


wheel_rl = car.AddWheels("Rear Left Wheel")
wheel_rl.SetName("Rear Left Wheel")
wheel_rl.SetRadius(0.3)
wheel_rl.SetMass(10)
wheel_rl.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
wheel_rl.SetPos(chrono.ChVector3d(0, 0, -0.5))
wheel_rl.SetRot(chrono.QuatFromAxisAngle(chrono.ChVector3d(1, 0, 0), math.radians(90)))


wheel_rr = car.AddWheels("Rear Right Wheel")
wheel_rr.SetName("Rear Right Wheel")
wheel_rr.SetRadius(0.3)
wheel_rr.SetMass(10)
wheel_rr.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
wheel_rr.SetPos(chrono.ChVector3d(-0.5, 0, -0.5))
wheel_rr.SetRot(chrono.QuatFromAxisAngle(chrono.ChVector3d(1, 0, 0), math.radians(-90)))


suspension = car.AddSuspension("Main Suspension")
suspension.SetName("Main Suspension")
suspension.SetLength(100)
suspension.SetDampingF(0.1)
suspension.SetSpringRate(1000)
suspension.SetCompliance(0.1)
suspension.SetPos(chrono.ChVector3d(0, 0, 0))
suspension.EnableCollision(True)


sys.Add(chassis)
sys.Add(wheel_fl)
sys.Add(wheel_fr)
sys.Add(wheel_rl)
sys.Add(wheel_rr)
sys.Add(suspension)


t = 0.0
time_step = 0.001
frame_rate = 50  


def driver_controls(delta_time):
    driver.steering = 0.5 * math.sin(t * 10)
    driver.throttle = 0.5 * math.cos(t * 10)
    driver.braking = 0.0


while True:
    t += time_step
    sys.DoStepDynamics(time_step)
    
    
    driver_controls(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if vis.Run() and t > 1 / (frame_rate / 1000):
        continue