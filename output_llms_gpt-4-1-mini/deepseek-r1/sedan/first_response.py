import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ----------------------------
# Initialize simulation system
# ----------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverMaxIterations(100)

# ------------------
# Create the vehicle
# ------------------
vehicle = veh.Sedan()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# -----------------
# Create TMeasy tires
# -----------------
tire = veh.TMeasyTire("tire")
tire.SetParameters("tmeasy/tire/Sedan_TMeasyTire.json")
vehicle.InitializeTire(tire, vehicle.GetAxle(0).m_wheels[0], veh.VisualizationType_MESH)

# Repeat for other wheels
for axle in [0, 1]:
    for wheel in [0, 1]:
        vehicle.InitializeTire(veh.TMeasyTire("tire"), 
                             vehicle.GetAxle(axle).m_wheels[wheel], 
                             veh.VisualizationType_MESH)

# --------------------------
# Create and configure terrain
# --------------------------
terrain = veh.RigidTerrain(system, "terrain")
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(200, 200, 1), 
                        chrono.ChVectorD(0, 0, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Add logo texture patch
logo_patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(5, 5, 0.02)), 
                             chrono.ChVectorD(10, 10, 0.1), 
                             chrono.ChVectorD(0, 0, 1))
logo_patch.SetTexture(veh.GetDataFile("textures/logo_chronoengine.png"))
terrain.Initialize()

# --------------------------
# Create Irrlicht visualization
# --------------------------
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('BMW E90 Sedan Dynamics')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 6, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), 
                      chrono.ChVectorD(0, 0, 0), 
                      50, 5, 50, 512, 
                      chrono.ChColor(1, 1, 1))

# Chase camera setup
tracker = irr.ChChaseCamera(vis.GetCamera())
tracker.SetCamera(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 5, 2))
tracker.SetState(irr.ChChaseCamera_Chase)
tracker.SetStep(0.01)

# --------------------------
# Create interactive driver
# --------------------------
driver = veh.InteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.SetInputDataFile("driver_inputs.txt")
driver.SetInputMode(veh.InputMode_DATAFILE)

# --------------------------
# Simulation loop
# --------------------------
vehicle.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = system.GetChTime()
    
    # Update visualization and inputs
    vis.BeginScene()
    vis.Render()
    tracker.Update(vehicle.GetVehicle().GetChassisBody())
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    vis.Synchronize("", driver.GetInputs())
    vis.EndScene()
    
    # Advance simulation
    step = 0.01
    system.DoStepDynamics(step)

    # Update HUD
    speed = vehicle.GetVehicle().GetSpeed()
    vis.GetGUIEnvironment().addStaticText(
        f"Speed: {speed:.2f} m/s\nThrottle: {driver.GetThrottle():.2f}\nBraking: {driver.GetBraking():.2f}",
        irr.recti(10, 10, 200, 60))