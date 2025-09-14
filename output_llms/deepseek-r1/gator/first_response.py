import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================
# Initialize simulation environment
# =============================================================================

# Create Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], ''))

# =============================================================================
# Create vehicle and terrain
# =============================================================================

# Initialize Gator vehicle
gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.Initialize()

# Set visualization modes
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

# Create rigid terrain
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(100, 100, 0.2), 
                        chrono.ChVectorD(0, 0, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# =============================================================================
# Set up visualization and interactive driver
# =============================================================================

# Create Irrlicht app
app = irr.ChIrrApp(system, "Gator Simulation", irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 6, 1.5))
app.SetTimestep(0.001)

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(app.GetDevice())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# =============================================================================
# Real-time simulation loop
# =============================================================================

app.AssetBindAll()
app.AssetUpdateAll()

realtime_timer = chrono.ChRealtimeStepTimer()
time_step = 0.001
frame_time = 1/50.0  # 50 FPS
elapsed_time = 0.0

while app.GetDevice().run():
    time = system.GetChTime()
    
    # Update visualization and inputs
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.GetDevice().getGUIEnvironment().drawAll()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    gator.DriverInputs(driver_inputs)
    
    # Update modules
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)
    
    # Advance simulation
    app.DoStepDynamics(time_step)
    
    # Maintain real-time synchronization
    elapsed_time += time_step
    realtime_timer.Spin(time_step)
    
    # Control rendering frequency
    if elapsed_time >= frame_time:
        app.EndScene()
        elapsed_time = 0.0

# Properly clean up resources
app.GetDevice().closeDevice()