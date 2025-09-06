import chrono
import pychrono
import numpy as np
import matplotlib.pyplot as plt
from pychrono.robot import Robot
from pychrono.vehicle import Vehicle
from pychrono.irrlicht import Irrlicht
import time

# 1. Initialize PyChrono Environment
try:
    chrono = chrono.ChSystemNSC()
    print("PyChrono environment initialized successfully.")
except Exception as e:
    print(f"Error initializing Chrono environment: {e}")
    exit()


# 2. Define Physical Systems & Objects
# CityBus - Basic Vehicle
city_bus = Vehicle()
city_bus.SetContactMethod(contact_method)
city_bus.SetChassisCollisionType(chassis_collision_type)
city_bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI), 100)) # Initial position
city_bus.SetInitialOrientation(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI))) # Initial orientation
city_bus.SetTireModel(chrono.ChTireModel_Rubber)
city_bus.SetTireRadius(chrono.ChVector3d(1.0, 0.5, 0.7)) # Tire radius
city_bus.SetTirePressure(chrono.ChPressure(0.6)) # Tire pressure
city_bus.SetVehicleType(chrono.ChVehicleType_Bus)

# 3.  Set Default Collision Parameters
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)

# 4. Visualization Setup
vis = Irrlicht.initialize()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("CityBus Simulation")
vis.AddCamera(chrono.ChVector3d(0, 0, 0)) # Camera position
vis.AddLight(chrono.ChLight(chrono.ChColor(0.5, 0.5, 0.5))) # Light color
vis.AddTexture(chrono.ChTexture(chrono.ChColor(0.7, 0.7, 0.7))) # Background texture
vis.AddShape(chrono.ChShape(chrono.ChSolidColor(chrono.ChColor(0.8, 0.8, 0.8)))) # Bus shape
vis.AddNode(city_bus)

# 5.  Set Initial Contact Settings (Simplified)
chrono.ChContactContainer.RegisterAddContactCallback(city_bus.GetContactContainer().ReportContactCallback)

# 6.  Create a Rigid Terrain
terrain = chrono.ChTerrain()
terrain.SetRigidTerrain(chrono.ChTerrainType_Terrain)
terrain.SetTexture(chrono.ChTexture(chrono.ChColor(0.6, 0.6, 0.6))) # Grass texture
terrain.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI), 100))
terrain.SetInitialOrientation(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
terrain.SetPatch(chrono.ChPatch(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI), 100), chrono.ChTexture(chrono.ChColor(0.6, 0.6, 0.6))) # Patch
terrain.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI), 100))
terrain.SetInitialOrientation(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
terrain.SetTerrainHeight(chrono.ChHeight(100))
terrain.SetInitialTexture(chrono.ChTexture(chrono.ChColor(0.6, 0.6, 0.6)))


# 7.  Simulation Loop
print("Simulating CityBus...")
try:
    while True:
        time.sleep(50)
        #Update the simulation
        city_bus.SetPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI), 100))
        city_bus.SetOrientation(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
        city_bus.SetTirePressure(chrono.ChPressure(0.6))
        city_bus.SetVehicleType(chrono.ChVehicleType_Bus)

        vis.Clear()
        vis.AddNode(city_bus)
        vis.Draw()

except Exception as e:
    print(f"Simulation Error: {e}")

# 8.  Exit Simulation
print("Simulation finished.")