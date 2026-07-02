import argparse
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr

# Initialize PyChrono
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Create terrain
terrain = veh.ChTerrain(system)
terrain.SetLength(100)
terrain.SetHeightFunction(veh.ChHeightFunctionPlane(0, 0, 0))
terrain.SetMaterial(veh.ChMaterialSurfaceNSC(0.01, 0.01))
terrain.Initialize(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(100, 100, 0))
system.Add(terrain)

# Create Gator vehicle
vehicle = veh.ChGator(system)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_NONE)
vehicle.Initialize(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0), True)
system.Add(vehicle)

# Create driver
driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()
driver.SetThrottle(0.0)
driver.SetSteering(0.0)
driver.SetBraking(0.0)

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetAmbientLightIntensity(0.5)
sensor_manager.SetUseRaytracing(False)

# Create point lights
light1 = sens.ChPointLight(chrono.ChVector3d(0, 0, 2), 10000000, 1.0, 0.2, 0.1)
light2 = sens.ChPointLight(chrono.ChVector3d(0, 0, -2), 10000000, 1.0, 0.2, 0.1)
sensor_manager.AddSensor(light1)
sensor_manager.AddSensor(light2)

# Create camera
camera = sens.ChCameraSensor(vehicle.GetChassisBody(), True, 1.0, 100)
camera.SetName("Camera")
camera.SetUpdateRate(10)
camera.SetOffset(chrono.ChVector3d(0, 0, 2))
camera.SetLookAt(chrono.ChVector3d(0, 0, 0))
camera.SetUp(chrono.ChVector3d(0, 0, 1))
sensor_manager.AddSensor(camera)

# Create visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Gator Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# Simulation loop
time = 0
step = 0
while vis.Run() and vis.GetKeyboardSwitch() == irr.KeyboardSwitch.NONE:
    time += 1e-3
    step += 1

    # Update driver
    driver.Synchronize(time)
    driver.Advance(time)

    # Update vehicle
    vehicle.Synchronize(time, driver.GetInputs())
    vehicle.Advance(time)

    # Update terrain
    terrain.Synchronize(time)
    terrain.Advance(time)

    # Update sensors
    sensor_manager.Update()

    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    chrono.ChSystem::IncrementTime(system, 1e-3)