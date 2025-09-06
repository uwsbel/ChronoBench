# Import necessary libraries
import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.robot as robosimian

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', 'data'))

# Initialize the system and visualization
my_system = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Initialize the Gator vehicle
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(chrono.ChCollisionModel.CollisionType_SURFACE)
gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
gator.Initialize()

# Initialize the rigid terrain
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Initialize the interactive driver system
driver = robosimian.RS_Driver(0, 0, 0, True)
driver.SetVehicle(gator)

# Initialize the sensor manager with point lights and a camera
sensor_manager = robosimian.RS_SensorManager()
point_light = chrono.ChLightPoint()
point_light.SetColor(chrono.ChColor(1, 1, 1))
point_light.SetAttenuation(0.5, 0.01, 0.001)
point_light.SetPos(chrono.ChVector3d(0, 0, 10))
sensor_manager.AddLight(point_light)
camera = chrono.ChCamera()
camera.SetPos(chrono.ChVector3d(0, 0, 5))
camera.SetDir(chrono.ChVector3d(0, 0, -1))
camera.SetFOV(math.atan(0.5 / 5) * 2)
sensor_manager.AddCamera(camera)
gator.GetChassis().AddSensorManager(sensor_manager)

# Simulation loop
while vis.Run():
    # Update the driver, terrain, vehicle, and sensor manager
    driver.Update(0.01)
    terrain.Advance(0.01)
    gator.Advance(0.01)
    sensor_manager.Advance(0.01)

    # Synchronize and advance the simulation modules
    my_system.DoStepDynamics(0.01)
    vis.RunCamera(camera)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Clean up and exit
vis.Cleanup()