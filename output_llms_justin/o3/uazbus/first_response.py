# =============================================================================
#  UAZBUS          : minimal, self–contained PyChrono 7.x demo
# =============================================================================
#  – Rigid-body terrain with user-specified friction & restitution
#  – Interactive keyboard driver (steer/throttle/brake)
#  – Irrlicht real-time visualization with lights & chase camera
# =============================================================================

import pychrono                         as chrono
import pychrono.vehicle                 as veh
import pychrono.irrlicht                as irr

# ---------------------------------------------------------------------------
# 1. General simulation parameters
# ---------------------------------------------------------------------------
step_size       = 2e-3          # s
t_end           = 60.0          # s, safety exit (Esc exits anytime)
gravity         = chrono.ChVectorD(0, 0, -9.81)
contact_fric    = 0.9           # terrain friction coefficient
contact_rest    = 0.1           # terrain restitution coefficient

# ---------------------------------------------------------------------------
# 2. Initialize Chrono (paths, system, timer)
# ---------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())         # reuse packaged data
system      = chrono.ChSystemSMC()
system.Set_G_acc(gravity)

# A real-time timer to keep the simulation in sync with wall clock
realtime_timer = chrono.ChRealtimeStepTimer()

# ---------------------------------------------------------------------------
# 3. Create the vehicle (UAZBUS)                                                  
# ---------------------------------------------------------------------------
init_pos   = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)  # x-y-z, level

vehicle = veh.WheeledVehicle(system,
                              veh.GetDataFile("vehicle/uaz/UAZBUS.json"),
                              veh.ChContactMethod_SMC)

vehicle.Initialize(init_pos)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Power-train
powertrain = veh.ShaftsPowertrain(veh.GetDataFile("vehicle/uaz/UAZBUS_ShaftsPowertrain.json"))
vehicle.InitializePowertrain(powertrain)

# ---------------------------------------------------------------------------
# 4. Rigid Terrain
# ---------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch   = terrain.AddPatch(chrono.ChCoordsysD(),            # position/orientation
                           chrono.ChVectorD(400, 400, 1),   # size (x,y,z)
                           0,                               # height
                           True)                            # tiled texture

patch.SetContactFrictionCoefficient(contact_fric)
patch.SetContactRestitutionCoefficient(contact_rest)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()

# ---------------------------------------------------------------------------
# 5. Driver (keyboard interactive)
# ---------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(vehicle.GetSystem())   # uses Irrlicht event receiver
driver.SetSteeringDelta( 1.0 * chrono.CH_2PI / 180)    # rad per key-press
driver.SetThrottleDelta( 0.1)
driver.SetBrakingDelta(  0.2)

# ---------------------------------------------------------------------------
# 6. Irrlicht application (visual system)
# ---------------------------------------------------------------------------
app = veh.ChWheeledVehicleIrrApp(vehicle,
                                 "UAZBUS real-time demo",
                                 irr.dimension2du(1280, 720))

app.SetSkyBox()
app.AddTypicalLights(chrono.ChVectorD( 30,  30, 60), 300, chrono.ChColor(1,1,1))
app.AddTypicalLights(chrono.ChVectorD(-30, -30, 60), 300, chrono.ChColor(1,1,1))
app.SetChaseCamera(chrono.ChVectorD(0, 0, 1.75),      # look-at point on chassis
                   6.0,                               # chase distance
                   0.5)                               # elevation
app.Initialize()

# ---------------------------------------------------------------------------
# 7. Main simulation loop
# ---------------------------------------------------------------------------
print("Controls:  arrow L/R = steer, arrow U/D = throttle/brake, SPACE = full brake, q=quit")

time = 0.0
while app.GetDevice().run() and time < t_end:

    # ---- input from driver --------------------------------------------------
    app.BeginScene()
    app.DrawAll()               # also feeds the GUI driver with keyboard events
    app.EndScene()

    # Synchronize all modules
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs.steering, driver_inputs.throttle, driver_inputs.braking)
    terrain.Synchronize(time)
    driver.Synchronize(time)

    # Advance dynamics and modules
    system.DoStepDynamics(step_size)
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    # Spin in real-time
    realtime_timer.Spin(step_size)

    time += step_size

print("Execution finished.")