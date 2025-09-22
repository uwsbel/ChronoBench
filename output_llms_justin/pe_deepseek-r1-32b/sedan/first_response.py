import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.robot as rob
import pychrono.irrlicht as chronoirr

# Set up the Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "data"))

# Create the simulation system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Create the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("BMW E90 Sedan Simulation")
vis.AttachSystem(sys)
vis.SetRenderMode(chronoirr.RENDER_MODE_OPENGL)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()

# Create the vehicle
veh_dir = os.path.join(chrono.GetChronoDataPath(), "vehicle")
bmw_e90_model = os.path.join(veh_dir, "BMW_E90 Sedan", "BMW_E90 Sedan chassis.obj")
if not os.path.exists(bmw_e90_model):
    raise FileNotFoundError("BMW E90 model file not found")

vehicle = veh.ChVehicle()
vehicle.SetContactMethod(chrono.ChContactMethod.NSC)
vehicle.SetChassisCollisionType(veh.CollisionType.RIGID)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.Initialize()

# Create the terrain
terrain = veh.ChRigidTerrain(sys)
patch = terrain.AddPatch(veh.ChTireMaterialRigid(), chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetTexture(chrono.GetChronoDataFile("textures/checkerboard.jpg"), 10, 10)
terrain.Initialize()

# Create the driver system
driver = rob.ChInteractiveDriver(vehicle.GetDriver(), True, True, True)
driver.SetMaxThrottle(1.0)
driver.SetMaxBraking(1.0)
driver.SetMaxSteering(0.5)
vehicle.SetDriver(driver)

# Add visualization components
chase_cam = vis.Get_Camera()
chase_cam.SetChaseVehicle(vehicle, 5, 2, 2)
vis.AddDirectionalLight(chrono.ChVectorD(0, -1, 0), chrono.ChColor(0.8, 0.8, 0.8))

# Add the vehicle and terrain to the system
sys.Add(vehicle)
sys.Add(terrain)

# Run the simulation
while vis.Run():
    driver.Synchronize()
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.Close()