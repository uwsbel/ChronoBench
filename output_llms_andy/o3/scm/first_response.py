#!/usr/bin/env python3
"""
HMMWV on SCM deformable terrain – interactive, real–time simulation.

• Full HMMWV model (suspension, powertrain, steering, etc.)
• RIGID tires – mesh visualisation for every vehicle component
• SCM (Soil Contact Model) deformable terrain with custom soil parameters
  and a moving patch that tracks the chassis to keep the node count modest
• False–colour sinkage plot on the terrain
• Irrlicht visualisation + interactive keyboard driver
• Real-time loop, rendered at 50 frames/second
"""

# ---------------------------------------------------------------------
# 1. Imports and Chrono initialisation
# ---------------------------------------------------------------------
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Helpful Chrono utilities
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ---------------------------------------------------------------------
# 2. Global simulation parameters
# ---------------------------------------------------------------------
# Desired real-time rendering frame-rate
RENDER_FPS        = 50            # [frames/s]
RENDER_STEP_SIZE  = 1.0 / RENDER_FPS
# Internal integrator step
STEP_SIZE         = 1e-3          # [s]  – physics step
# Terrain dimensions and node spacing for SCM grid
TERRAIN_LENGTH    = 80.0          # [m]  size of initial patch along x
TERRAIN_WIDTH     = 80.0          # [m]  size of initial patch along y
NODE_SPACING      = 0.1           # [m]  SCM grid spacing

# Vehicle initial configuration
INIT_LOC          = chrono.ChVectorD(0.0, 0.0, 0.5)          # x,y,z [m]
INIT_ROT          = chrono.ChQuaternionD(1, 0, 0, 0)          # no yaw/pitch/roll
TIRE_STEP_SIZE    = STEP_SIZE                                 # step used inside tire model

# ---------------------------------------------------------------------
# 3. Create the HMMWV vehicle (full, 4 wheels, rigid tires)
# ---------------------------------------------------------------------
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetPowertrainType(veh.PowertrainType_SHAFTS)
hmmwv.SetDriveType(veh.DriveType_AWD)
hmmwv.SetTireType(veh.TireType_RIGID)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)

# Add mesh visualisation to every sub-system
hmmwv.SetChassisVisualizationType     (veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType  (veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType    (veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType       (veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType        (veh.VisualizationType_MESH)

hmmwv.Initialize()

system = hmmwv.GetSystem()        # Grab the underlying Chrono system

# ---------------------------------------------------------------------
# 4. SCM deformable terrain
# ---------------------------------------------------------------------
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVectorD(0,0,0), chrono.QUNIT))
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, NODE_SPACING)

# Soil parameters  (values chosen as illustration – feel free to tune)
Bekker_Kphi     = 5301e3     # [N/m^(n+1)]
Bekker_Kc       = 102e3      # [N/m^(n+1)]
Bekker_n        = 1.1        # exponent
Mohr_cohesion   = 1.0e3      # [Pa]
Mohr_friction   = 30.0       # [deg] internal friction angle
Janosi_shear    = 0.02       # [m]  shear coefficient

terrain.SetSoilParameters(Bekker_Kphi, Bekker_Kc, Bekker_n,
                          Mohr_cohesion, Mohr_friction, Janosi_shear)

# Optional – a more “rubbery” look
terrain.SetElasticModulus(2e8)          # [Pa]
terrain.SetDamping(0.01)                # dimensionless Rayleigh damping

# Moving patch that follows the chassis (keeps node count low)
terrain.AddMovingPatch(hmmwv.GetChassisBody(),
                       chrono.ChVectorD( 0, 0, 0),
                       chrono.ChVectorD( 5, 5, 1))     # (Lx, Ly, Lz) patch half-sizes

# False-colour plot: sinkage (blue=small, red=large)
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.2)

# ---------------------------------------------------------------------
# 5. Irrlicht application and interactive driver
# ---------------------------------------------------------------------
app = veh.ChWheeledVehicleIrrApp(hmmwv, "HMMWV on SCM Deformable Terrain",
                                 irr.dimension2du(1280, 720))

app.SetSkyBox()
app.AddTypicalLogo()
app.AddTypicalLights(irr.vector3df(30, 100, 30),     # point light
                     irr.vector3df(-30, 100, 30),    # second point light
                     250.0, 130.0)
app.AddTypicalCamera(irr.vector3df(1,1,1))           # needed before chase-cam set-up
app.SetChaseCamera(chrono.ChVectorD(0, 0, 0.5), 6.0, 0.5)

# Attach a GUI driver that maps keyboard input to steering/throttle/brakes
driver = veh.ChIrrGuiDriver(app,  # Irrlicht app
                            0.3,  # steering time constant
                            0.01, # throttle time constant
                            0.3)  # braking time constant
driver.Initialize()

# Keep Chrono in real time
realtime_timer = chrono.ChRealtimeStepTimer()

# ---------------------------------------------------------------------
# 6. Main simulation loop
# ---------------------------------------------------------------------
render_steps = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)
step_counter = 0

print("Simulation running.  Use arrow keys (steer), SPACE (brake), and")
print("keys W/S for throttle.  Close the Irrlicht window to exit.")

while app.GetDevice().run():
    # 6.1 Render scene at the desired FPS
    if step_counter % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

    # 6.2 Collect input from the interactive driver
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()

    # 6.3 Synchronise modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)

    # 6.4 Advance simulation for one step
    driver.Advance(STEP_SIZE)
    terrain.Advance(STEP_SIZE)
    hmmwv.Advance(STEP_SIZE)
    app.Advance(STEP_SIZE)

    # 6.5 Enforce real-time
    realtime_timer.Spin(STEP_SIZE)

    step_counter += 1