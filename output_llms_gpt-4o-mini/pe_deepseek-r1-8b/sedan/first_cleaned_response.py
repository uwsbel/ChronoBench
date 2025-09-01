import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, vehicle, irrlight, fea


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


car = vehicle.VEHICLE_TYPEsedan()
car.SetName("BMW E90")
car.SetMass(1400)  
car.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  
car.SetFixed(True)  


car_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2.2, 1.8, 1.4))  
car_shape.SetColor(chrono.ChColor(255, 255, 255))  
car.AddVisualShape(car_shape)


tire = vehicle.TMEASY()
tire.SetNumTires(4)
tire.SetTireRadius(0.35)
tire.SetTireWidth(0.35)
car.Add(tire)


terrain = vehicle.RigidTerrain(sys)
terrain.SetMaterial(chrono.ChMaterial())
terrain.GetMaterial().SetFriction(0.6)  
terrain.GetMaterial().SetDampingF(0.3)  


patch = terrain.AddPatch(
    "road",
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    50, 10
)


logo_path = os.path.join(os.path.dirname(__file__), "logo_pychrono_alpha.png")
terrain.AddLogo(logo_path)


suspension = vehicle.SUSPENSION_TYPEdouble_wishbone()
suspension.SetSpringRate(1000)
suspension.SetDampingF(5)
suspension.SetLeverArm(0.5)


car.Add(suspension)


drivetrain = vehicle.DRIVETRAIN_TYPEmanual()
drivetrain.SetSteeringRatio(0.5)
drivetrain.SetPower(100)


car.Add(drivetrain)


suspension.Initialize(car, car, chrono.ChCoordsysd(chrono.ChVector3d(0, 1, 0)))
drivetrain.Initialize(car, car, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0)))


collision_model = chrono.ChCollisionModel(sys)
collision_model.SetDefaultSuggestedEnvelope(0.01)
collision_model.SetDefaultSuggestedMargin(0.005)


car.EnableCollision(True)


driver = vehicle.DRIVER_TYPEmanual()
driver.SetMaxSteeringAngle(30)
driver.SetMaxThrottle(1)
driver.SetMaxBrake(1)


car.SetDriver(driver)


vis = irrlight.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("BMW E90 Sedan Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  


vis.AddTypicalLights()
vis.AddSkyBox()


time_step = 0.01


steer_func = chrono.ChFunction_Sine(0.1, 0.5)  
throttle_func = chrono.ChFunction_Const(1.0)  
brake_func = chrono.ChFunction_Const(1.0)  


motor_steering = chrono.ChLinkMotorRotationSpeed()
motor_steering.Initialize(car, car, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  
motor_throttle = chrono.ChLinkMotorRotationSpeed()
motor_throttle.Initialize(car, car, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  
motor_brake = chrono.ChLinkMotorRotationSpeed()
motor_brake.Initialize(car, car, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  


motor_steering.SetSpeedFunction(steer_func)
motor_throttle.SetSpeedFunction(throttle_func)
motor_brake.SetSpeedFunction(brake_func)


while True:
    
    sys.DoStepDynamics(time_step)
    
    
    driver.Control = driver.GetDriverInput()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if not vis.Run():
        break


sys_fea = fea.ChSystem()
beam = fea.ChBeamSectionEulerAdvanced()
beam.SetAsRectangularSection(0.1, 0.1)
beam.SetYoungModulus(0.01e9)
sys_fea.Add(beam)


sys_mbs = chrono.ChSystemNSC()
table = chrono.ChBody()
table.SetPos(chrono.ChVector3d(0, -0.5, 0))
table_shape = chrono.ChVisualShapeBox(1, 0.5, 0.5)
table.AddVisualShape(table_shape)
sys_mbs.Add(table)
shaker = chrono.ChLinkLockPrismatic()
shaker.Initialize(table, table_floor, frame)
sys_mbs.Add(shaker)
shaker_motor = chrono.ChLinkMotorRotationSpeed()
shaker_motor.Initialize(table, table_floor, frame)
shaker_motor.SetSpeedFunction(chrono.ChFunction_Sine(0.1, 1.5))