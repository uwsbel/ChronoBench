import chrono
import pychrono
import numpy as np
import os
import math


try:
    chrono = chrono.ChSystem()
    chrono.SetChronoDataPath('/path/to/data') 
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()


try:
    from pychrono.robot import RoboSimian
    from pychrono.irrlicht import Visualization
    from pychrono.vehicle import Vehicle
except ImportError as e:
    print(f"Error importing required modules: {e}")
    exit()




mesh_path = 'triangular_mesh.obj'  
try:
    mesh = RoboSimian(chrono, True, True, mesh_path)
    print(f"Mesh loaded successfully: {mesh.GetSystem()}")
except Exception as e:
    print(f"Error loading mesh: {e}")
    exit()



collision_system = chrono.ChCollisionSystem.Type_BULLET


contact_method = chrono.ChContactMethod.Type_Euler


contact_radius = 0.01


damping = 0.000


compliance = 0.01


simulation_time_step = 0.01


camera_sensor = chrono.ChSensor()
camera_sensor.SetCamera(chrono.ChVector3d(0, 0, 0))
camera_sensor.SetNoiseFilters(chrono.ChNoiseFilter.Type_Linear)
camera_sensor.SetVisualization(chrono.ChVisualization.Type_Irrlicht)


try:
    visualization = Visualization(chrono, camera_sensor)
    print(f"Visualization initialized successfully: {visualization.GetSystem()}")
except Exception as e:
    print(f"Error initializing Visualization: {e}")
    exit()


try:
    body = mesh.GetBody()
    print(f"Body initialized: {body.GetSystem()}")
except Exception as e:
    print(f"Error initializing body: {e}")
    exit()


try:
    
    rev_joint = body.GetJoint()
    rev_joint.SetRotation(chrono.ChVector3d(0, 0, 0)) 
    print(f"Revolute Joint initialized: {rev_joint.GetSystem()}")

    
    prismatic_joint = body.GetJoint()
    prismatic_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print(f"Prismatic Joint initialized: {prismatic_joint.GetSystem()}")

    
    spherical_joint = body.GetJoint()
    spherical_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print(f"Spherical Joint initialized: {spherical_joint.GetSystem()}")

    
    universal_joint = body.GetJoint()
    universal_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print(f"Universal Joint initialized: {universal_joint.GetSystem()}")

    
    print("Simulating the system...")
    sys.DoStepDynamics(simulation_time_step)

except Exception as e:
    print(f"Error during body initialization: {e}")
    exit()


try:
    while visualization.Run():
        print(f"Visualization iteration: {visualization.GetIteration()}")
        
        sys.DoStepDynamics(simulation_time_step)

except Exception as e:
    print(f"Error during visualization loop: {e}")
    exit()