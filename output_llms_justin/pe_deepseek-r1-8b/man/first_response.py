import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, vehicle, irrrlicht, fea

# Initialize the PyChrono system with SMC for smooth contact handling
sys = chrono.ChSystemSMC()

# Set the path to Chrono's data directory
chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Initialize the vehicle model
from pychrono.vehicle import ARTcar

# Create the vehicle and set initial parameters
car = ARTcar(sys)
car.SetContactMethod(chrono.ChCollisionSystem.Type_NS_C)
car.SetChassisCollisionType(chrono.ChVehicleCollisionType.Rigid)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
car.Initialize()

# Add the vehicle to the system
sys.Add(car)

# Create the terrain
terrain = vehicle.RigidTerrain(car.GetSystem())
terrain.Initialize()

# Add road patches to the terrain
patch_material = chrono.ChMaterial()
patch_material.SetFriction(0.8)
patch_material.SetDampingF(0.2)
patch = terrain.AddPatch(
    patch_material,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    50,  // terrain length
    10   // terrain width
)
terrain.AddPatch(
    patch_material,
    chrono.ChCoordsysd(chrono.ChVector3d(100, 0, 0), chrono.QUNIT),
    50,
    10
)

# Initialize the visualization system
vis = irrrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.AddLogo(chrono.GetChronoDataFile('pychrono-logo.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  # Chase camera
vis.AddTypicalLights()

# Set up the camera
cam = vis.GetCamera()
cam.SetPosition(chrono.ChVector3d(0, 5, 10))
cam.SetRotation(chrono.QuatFromAxisEuler(chrono.CH_PI/2, 0, 0))  # Look back

# Set up the tires
num_tires = 4
tire_radius = 0.4
wheelbase = 2.5

for i in range(num_tires):
    tire = car.GetTires()[i]
    tire.SetTireModel(chrono.ChTireModel.TMEASY)
    tire.SetTireRadius(tire_radius)
    tire.SetWheelbase(wheelbase)
    tire.SetSuspensionStiffness(100000)
    tire.SetDampingF(0.1)
    tire.EnableCollision(True)

# Set up real-time controls
def control_function_throttle(time, speed):
    return speed  # Simple speed control

def control_function_steering(time, speed, angle):
    return speed * np.clip(angle, -1, 1)  # Proportional steering

def control_function_brake(time, speed):
    return -speed * 0.1  # Proportional braking

# Attach control functions to the vehicle
car.SetControlFunctionThrottle(control_function_throttle)
car.SetControlFunctionSteering(control_function_steering)
car.SetControlFunctionBrake(control_function_brake)

# Add all components to the system
sys.Add(car)
sys.Add(terrain)

# Initialize the simulation
sys.Initialize()

# Define the simulation loop
t = 0
dt = 0.01

while True:
    if vis.Run():
        # Render the scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Update the simulation
        sys.DoStepDynamics(dt)
        t += dt
        
        # Update the visualization
        vis.Update()
    
    # Apply real-time controls
    car.UpdateControls(t, dt)
    
    # Print simulation time
    if t > 0 and t % 1 == 0:
        print(f"Simulation Time: {t:.3f} seconds")