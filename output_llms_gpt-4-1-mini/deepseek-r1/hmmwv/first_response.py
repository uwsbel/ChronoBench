import os
import math
import time
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ----------------------------
# Initialize simulation system
# ----------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverMaxIterations(150)

# ------------------
# Create the terrain
# ------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(200, 200, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# -----------------
# Create the HMMWV
# -----------------
hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), 
                                        chrono.Q_from_AngY(0)))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(0.001)
hmmwv.Initialize()

# Set visualization modes
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# ------------------------
# Create Irrlicht viewer
# ------------------------
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 3, 1.5))
vis.AddTypicalLights()

# ---------------------
# Create driver system
# ---------------------
driver = veh.ChIrrGuiDriver(vis)

# Set steering parameters
driver.SetSteeringDelta(0.06)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# --------------------------
# Simulation loop parameters
# --------------------------
step_size = 0.002
render_step = 1.0 / 50  # 50 FPS
realtime_timer = chrono.ChRealtimeStepTimer()

# Main simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time synchronization
    realtime_timer.Spin(step_size)

    # Display vehicle stats
    vis.Synchronize(f'{hmmwv.GetVehicle().GetVehicleSpeed():.2f} m/s', driver_inputs)