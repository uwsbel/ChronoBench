#!/usr/bin/env python3
# ----------------------------------------------------------
# PyChrono ARTcar – rigid-terrain example with Irrlicht GUI
# ----------------------------------------------------------
import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ---------------------------------------------------------------------
# 1. Global set-up
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataDir())            # standard Chrono data
veh.SetDataPath(chrono.GetChronoDataDir() + "vehicle/")        # vehicle JSON files

# Simulation-step and render-step
step_size      = 1.0e-3                                        # integration step (s)
render_fps     = 50                                            # 50 FPS for visuals
render_interval= 1.0 / float(render_fps)                       # time between frames
render_steps   = math.ceil(render_interval / step_size)

# ---------------------------------------------------------------------
# 2. Create the Chrono physical system – choose a contact method
# ---------------------------------------------------------------------
contact_method = chrono.ChContactMethod_NSC
system         = chrono.ChSystemNSC() if contact_method==chrono.ChContactMethod_NSC \
                 else chrono.ChSystemSMC()

# ---------------------------------------------------------------------
# 3. Construct the ARTcar vehicle
# ---------------------------------------------------------------------
init_loc     = chrono.ChVectorD(0, 0, 0.1)                     # x,y,z [m]
init_rot     = chrono.ChQuaternionD(1, 0, 0, 0)                # no yaw/pitch/roll

vehicle = veh.WheeledVehicle(
            system,
            veh.GetDataFile("ARTcar/ARTcar_Vehicle.json"),
            contact_method)

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))

# Power-train and tyres ------------------------------------------------
powertrain = veh.ShaftsPowertrain(system,
                veh.GetDataFile("ARTcar/ARTcar_Powertrain.json"))
vehicle.InitializePowertrain(powertrain)

tire_file = veh.GetDataFile("ARTcar/ARTcar_TMeasyTire.json")
for axle in range(vehicle.GetNumberAxles()):
    for side in range(2):
        vehicle.GetWheelBody(axle, side).SetCollide(True)
        tire = veh.TMeasyTire(tire_file)
        vehicle.InitializeTire(tire, axle, side)

# ---------------------------------------------------------------------
# 4. Rigid terrain with a custom texture
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch   = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0),chrono.QUNIT),
                           chrono.ChVectorD(60, 10, 0.2),       # length, width, thickness
                           False)                               # not tiled
patch.SetTexture(veh.GetDataFile("textures/5.jpg"), 10, 10)     # simple asphalt picture
terrain.Initialize()

# ---------------------------------------------------------------------
# 5. Irrlicht application and GUI driver
# ---------------------------------------------------------------------
app = veh.ChVehicleIrrApp(vehicle, "ARTcar on rigid terrain",
                          irr.dimension2du(1024,768))

app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.2), 6.0, 0.5)   # track from behind
app.Initialize()

# Driver – time constants determine pedal/filter smoothness
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(1.0 / 200.0)
driver.SetThrottleDelta(1.0 / 150.0)
driver.SetBrakingDelta (1.0 / 150.0)
driver.Initialize()

# ---------------------------------------------------------------------
# 6. Main simulation loop
# ---------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
frame          = 0

while app.GetDevice().run():
    # Render when its time (every render_steps integration steps)
    if frame % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255,140,161,192))
        app.DrawAll()
        app.EndScene()

    # Collect driver input ([-1..1] steering, [0..1] throttle, [0..1] braking)
    driver_inputs = driver.GetInputs()

    # Update modules ---------------------------------------------------
    time = system.GetChTime()
    driver.Synchronize (time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize  ("ARTcar demo", driver_inputs)

    # Advance dynamics -------------------------------------------------
    driver.Advance  (step_size)
    terrain.Advance (step_size)
    vehicle.Advance (step_size)
    app.Advance     (step_size)

    # Real-time pacing
    realtime_timer.Spin(step_size)

    frame += 1