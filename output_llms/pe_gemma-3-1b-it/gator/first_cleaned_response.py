import chrono
import numpy as np
import math
import os
import sys
import time


try:
    chrono = chrono.ChSystem()
    print("PyChrono environment initialized.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()



simulation_name = "Gator_Vehicle_Simulation"
simulation_type = "Gator"
terrain_size = 100
terrain_width = 50
terrain_height = 50
terrain_texture = "texture.png"  

try:
    
    my_system = chrono.ChSystemNSC()
    print("System initialized.")
except Exception as e:
    print(f"Error initializing system: {e}")
    exit()



vehicle = chrono.ChBody()
vehicle.SetPos(chrono.ChVector3d(0, 0, 0))
vehicle.SetMass(100)
vehicle.SetContactMethod(chrono.ChContactMaterialNSC())
vehicle.SetChrictionMaterial(chrono.ChContactMaterialNSC())
vehicle.SetDampingF(0.1)
vehicle.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
vehicle.SetRayleighDamping(0.000)
vehicle.SetFixed(True)
vehicle.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


try:
    
    rigid_terrain = chrono.ChTerrain()
    rigid_terrain.SetSize(terrain_size)
    rigid_terrain.SetWidth(terrain_width)
    rigid_terrain.SetHeight(terrain_height)
    rigid_terrain.SetTexture(terrain_texture)
    print("Terrain created.")
except Exception as e:
    print(f"Error creating terrain: {e}")
    exit()


try:
    
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
    print("Default envelope settings set.")
except Exception as e:
    print(f"Error setting default envelope: {e}")
    exit()


try:
    
    vis = chrono.ChVisualSystemIrrlicht()
    print("Visualization system initialized.")
except Exception as e:
    print(f"Error initializing visualization: {e}")
    exit()


try:
    
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(0, 0, 0))
    body.SetMass(100)
    body.SetContactMethod(chrono.ChContactMaterialNSC())
    body.SetChrictionMaterial(chrono.ChContactMaterialNSC())
    body.SetDampingF(0.1)
    body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
    body.SetRayleighDamping(0.000)
    body.SetFixed(True)
    body.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    print("Body initialized.")
except Exception as e:
    print(f"Error initializing body: {e}")
    exit()


try:
    
    rev_joint = chrono.ChLinkLockRevolute()
    print("Revolute joint added.")
except Exception as e:
    print(f"Error adding revolute joint: {e}")
    exit()

    
    pris_joint = chrono.ChLinkLockPrismatic()
    print("Prismatic joint added.")
except Exception as e:
    print(f"Error adding prismatic joint: {e}")
    exit()

    
    spher_joint = chrono.ChLinkLockSpherical()
    print("Spherical joint added.")
except Exception as e:
    print(f"Error adding spherical joint: {e}")
    exit()


try:
    
    sys.DoStepDynamics(0.01)
    print("Simulation loop started.")
except Exception as e:
    print(f"Error starting simulation loop: {e}")
    exit()


try:
    
    vis.Render()
    print("Visualization rendered.")
except Exception as e:
    print(f"Error rendering simulation: {e}")
    exit()


finally:
    
    if 'vis' in locals():
        vis.Close()
    if 'my_system' in locals():
        my_system.Close()
    print("Simulation complete.")