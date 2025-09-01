import os
   import math
   import numpy as np
   from pychrono import chrono, irrrlicht
   from pychrono.robot import *
   from pychrono.vehicle import *

# Create a system with Non-Smooth Contact (NSC) for realistic contact handling
   sys = chrono.ChSystemNSC()

   # Initialize the visualization system
   vis = chronoirr.ChVisualSystemIrrlicht()
   vis.SetWindowSize(1920, 1080)
   vis.SetWindowTitle('Gator Vehicle Simulation')
   vis.Initialize()

# Create a rigid terrain body
   terrain_body = chrono.ChBody()
   terrain_body.SetPos(chrono.ChVector3d(0, 0, 0))
   terrain_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(100, 10, 10))
   terrain_body.AddVisualShape(terrain_shape)
   terrain_body.EnableCollision(False)  # Terrain doesn't move
   sys.Add(terrain_body)

   # Add a custom texture to the terrain
   texture_path = os.path.join(chrono.GetChronoDataFile(), 'terrain_texture.png')
   vis.AddTexture(texture_path)

# Create and initialize the Gator vehicle
   vehicle = veh.Gator(sys, contact_method='VEHICLE')
   vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))  # Initial position
   vehicle.SetContactMethod('VEHICLE')
   vehicle.SetTireModel('TMEASY')  # Use easy-to-model tire
   sys.Add(vehicle)

   # Add visual components to the vehicle
   vehicle_body = vehicle.GetBody()
   vehicle_body.AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(4, 4, 4)))  # Body
   vehicleSuspension = vehicle.GetSuspension()
   vehicleSuspension.AddVisualShape(chrono.ChVisualShapeCylinder(0.5, 2, 0.5))  # Suspension
   vehicleWheels = vehicle.GetWheels()
   for wheel in vehicleWheels:
       wheel.AddVisualShape(chrono.ChVisualShapeSphere(0.5, 0.5, 0.5))  # Wheels

# Create an interactive driver
   driver = RobotDriver(sys, 'MANUAL')  # Manual control mode
   driver.SetControlSignals(['STEERING', 'THROTTLE', 'BRAKE'])
   driver.EnableKeyboardCallback(key_callback)  # Define key callback function

def simulation_loop():
       global time, frame
       time = sys.GetTime()
       sys.DoStepDynamics(0.01)  # 10ms time step

       # Visualization
       vis.BeginScene()
       vis.Render()
       vis.EndScene()

       # Check if the visualization should continue running
       if not vis.Run():
           return

   # Run the simulation in a separate thread for better performance
   import threading
   thread = threading.Thread(target=simulation_loop)
   thread.start()

def key_callback(key, scancode, modifiers):
       global driver
       speed = 0.1  # Maximum speed multiplier
       if key == ord('w'):  # Accelerate
           driver.SetThrottle(speed)
       elif key == ord('s'):  # Brake
           driver.SetBrake(1.0)
       elif key == ord('a'):  # Steering left
           driver.SetSteering(-0.5)
       elif key == ord('d'):  # Steering right
           driver.SetSteering(0.5)
       elif key == ord('f'):  # Forward
           driver.SetThrottle(0.7)
       elif key == ord('r'):  # Reverse
           driver.SetThrottle(-0.7)
       elif key == ord(' '):  # Neutral
           driver.SetThrottle(0.0)
           driver.SetBrake(0.0)
       driver.Update()

# Start the simulation
   simulation_loop()

   # Main loop to handle user input
   while True:
       key = chr(input().strip())
       if key == 'q':
           break