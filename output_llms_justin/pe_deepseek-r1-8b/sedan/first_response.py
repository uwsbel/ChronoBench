import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, vehicle, irrlight, fea

# Set the data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Initialize the system using Non-Smooth Contact (NSC)
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Define vehicle properties
car = vehicle.VEHICLE_TYPEsedan()
car.SetName("BMW E90")
car.SetMass(1400)  # Mass of the vehicle
car.SetInertiaXX(chrono.ChVector3d(0, 0, 0))  # Default inertia for the vehicle
car.SetFixed(True)  # Fix the vehicle in space

# Add visual shape
car_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(2.2, 1.8, 1.4))  # Dimensions of the car
car_shape.SetColor(chrono.ChColor(255, 255, 255))  # White color with black accents
car.AddVisualShape(car_shape)

# Set up tires using TMEASY model
tire = vehicle.TMEASY()
tire.SetNumTires(4)
tire.SetTireRadius(0.35)
tire.SetTireWidth(0.35)
car.Add(tire)

# Create a rigid terrain
terrain = vehicle.RigidTerrain(sys)
terrain.SetMaterial(chrono.ChMaterial())
terrain.GetMaterial().SetFriction(0.6)  # Friction coefficient
terrain.GetMaterial().SetDampingF(0.3)  # Damping factor

# Add a patch to the terrain
patch = terrain.AddPatch(
    "road",
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    50, 10
)

# Add a logo to the terrain
logo_path = os.path.join(os.path.dirname(__file__), "logo_pychrono_alpha.png")
terrain.AddLogo(logo_path)

# Define suspension joints
suspension = vehicle.SUSPENSION_TYPEdouble_wishbone()
suspension.SetSpringRate(1000)
suspension.SetDampingF(5)
suspension.SetLeverArm(0.5)

# Add suspension to the car
car.Add(suspension)

# Define drivetrain links
drivetrain = vehicle.DRIVETRAIN_TYPEmanual()
drivetrain.SetSteeringRatio(0.5)
drivetrain.SetPower(100)

# Add drivetrain to the car
car.Add(drivetrain)

# Initialize the suspension and drivetrain
suspension.Initialize(car, car, chrono.ChCoordsysd(chrono.ChVector3d(0, 1, 0)))
drivetrain.Initialize(car, car, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0)))

# Set up collision model
collision_model = chrono.ChCollisionModel(sys)
collision_model.SetDefaultSuggestedEnvelope(0.01)
collision_model.SetDefaultSuggestedMargin(0.005)

# Enable collision for the car
car.EnableCollision(True)

# Create an interactive driver system
driver = vehicle.DRIVER_TYPEmanual()
driver.SetMaxSteeringAngle(30)
driver.SetMaxThrottle(1)
driver.SetMaxBrake(1)

# Add the driver to the car
car.SetDriver(driver)

# Initialize visualization
vis = irrlight.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("BMW E90 Sedan Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  # Position camera

# Add lighting
vis.AddTypicalLights()
vis.AddSkyBox()

# Define the time step
time_step = 0.01

# Define control functions for the driver
steer_func = chrono.ChFunction_Sine(0.1, 0.5)  # Sine wave for steering
throttle_func = chrono.ChFunction_Const(1.0)  # Constant throttle
brake_func = chrono.ChFunction_Const(1.0)  # Constant braking

# Define motor functions
motor_steering = chrono.ChLinkMotorRotationSpeed()
motor_steering.Initialize(car, car, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Steering motor
motor_throttle = chrono.ChLinkMotorRotationSpeed()
motor_throttle.Initialize(car, car, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Throttle motor
motor_brake = chrono.ChLinkMotorRotationSpeed()
motor_brake.Initialize(car, car, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Brake motor

# Register the control functions
motor_steering.SetSpeedFunction(steer_func)
motor_throttle.SetSpeedFunction(throttle_func)
motor_brake.SetSpeedFunction(brake_func)

# Run the simulation
while True:
    # Advance the simulation
    sys.DoStepDynamics(time_step)
    
    # Check if the driver is controlling the car
    driver.Control = driver.GetDriverInput()
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Check if the simulation should continue
    if not vis.Run():
        break

# Example FEA beam simulation
sys_fea = fea.ChSystem()
beam = fea.ChBeamSectionEulerAdvanced()
beam.SetAsRectangularSection(0.1, 0.1)
beam.SetYoungModulus(0.01e9)
sys_fea.Add(beam)

# Example MBS shaking table simulation
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