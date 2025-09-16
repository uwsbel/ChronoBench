import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np

# 1. Initialize PyChrono environment
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Update this path

# 2. Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 3. Create the terrain
terrain = chrono.ChBodyEasyBox(20, 0.5, 20, 1000, True, True)
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, -0.5, 0))
terrain.GetVisualModel().ClearModel()
terrain.GetVisualModel().AddBox(20, 0.5, 20, chrono.ChVectorD(0, -0.5, 0),
                               chrono.ChQuaternionD(1, 0, 0, 0))
terrain.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.GetCollisionModel().ClearModel()
terrain.GetCollisionModel().AddBox(20, 0.5, 20, chrono.ChVectorD(0, -0.5, 0),
                                 chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(terrain)

# 4. Create the rover
# Create the vehicle system
vehicle = veh.WheelVehicle(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_vehicle.json"))
vehicle.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))

# Create the driver system
driver = veh.ChInteractiveDriverIRR()
driver.Initialize(vehicle)

# Create the terrain-vehicle contact material
mat = veh.ChTerrainContactMaterial()
mat.mu = 0.8  # coefficient of friction
mat.k = 1e5   # stiffness
mat.gamma = 50  # damping

# Create the terrain
terrain = veh.RigidTerrain(system)
terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(20, 0, 20),
                chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                chrono.GetChronoDataFile("textures/concrete.jpg"))
vehicle.SetTerrain(terrain, mat)

# 5. Visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0), chrono.ChVectorD(0, 0, 0))
vis.SetCameraMoveScale(0.005)
vis.AddLight(chrono.ChVectorD(5, 10, 5), chrono.ChVectorD(0, 0, 0), 5, chrono.ChColor(1, 1, 1))
vis.EnableShadows()

# 6. Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Apply steering and throttle
    vehicle.SetSteering(driver_inputs.m_steering)
    vehicle.SetThrottle(driver_inputs.m_throttle)
    vehicle.SetBrake(driver_inputs.m_braking)

    # Advance simulation
    system.DoStepDynamics(0.01)

# 7. Clean up
vis.Close()