"""ARTcar small-scale RC vehicle on flat rigid terrain (PyChrono 9.0.0, Irrlicht).

Models the ARTcar catalog wheeled vehicle (NSC contact system) driving forward
on a flat RigidTerrain patch with a TMEASY tire model. The powertrain/tire tuning
is set explicitly so the car accelerates more aggressively: a higher max motor
voltage ratio (0.26), a higher stall torque (0.4), and a lower tire rolling
resistance (0.03). A scripted ChDataDriver applies steering/throttle so the car
launches and drives straight ahead.

System type: NSC. Main bodies: ARTcar chassis + four wheels/tires, rigid terrain
patch. Expected behavior: the car starts at rest, then accelerates forward along
+X under full throttle, reaching a steady speed faster than the baseline tuning.
"""

import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Parameters === geometry / physics / vehicle tuning constants
init_loc = chrono.ChVector3d(0, 0, 0.5)          # spawn slightly above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
terrain_length = 100.0                            # rigid patch size in X
terrain_width = 100.0                             # rigid patch size in Y
track_point = chrono.ChVector3d(0.0, 0.0, 0.2)   # chase-camera target on chassis

step_size = 1e-3                                  # integration step
tire_step_size = step_size                        # tire sub-step
render_step_size = 1.0 / 50.0                      # 50 FPS render cadence
sim_end = 10.0                                    # bounded run length (s)

# Vehicle tuning (faster-accelerating ARTcar):
max_motor_voltage_ratio = 0.26                    # raised from baseline for more drive
stall_torque = 0.4                                # raised stall torque
tire_rolling_resistance = 0.03                    # lowered rolling resistance

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


# === Vehicle === ARTcar catalog wrapper (owns its ChSystemNSC)
car = veh.ARTcar()
car.SetContactMethod(chrono.ChContactMethod_NSC)     # rigid terrain -> NSC
car.SetChassisCollisionType(veh.CollisionType_NONE)
car.SetChassisFixed(False)                           # MANDATORY — fixed chassis won't move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
car.SetTireType(veh.TireModelType_TMEASY)            # rolling tire model
car.SetTireStepSize(tire_step_size)
car.SetMaxMotorVoltageRatio(max_motor_voltage_ratio)  # faster: higher voltage ratio
car.SetStallTorque(stall_torque)                      # faster: higher stall torque
car.SetTireRollingResistance(tire_rolling_resistance) # faster: lower rolling resistance
car.Initialize()

car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.ARTcar wrapper) ===
system = car.GetSystem()                             # ChSystemNSC owned by the wrapper
chassis = car.GetChassisBody()  # cache: fetched once, reused every step
# wheels/spindles: car.GetVehicle().GetAxle(i)...; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", car.GetVehicle().GetMass())  # report total vehicle mass

# === Terrain === flat rigid patch the car drives on
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length, terrain_width,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === full vehicle Irrlicht scene: window + chase cam + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(car.GetVehicle())

# === Driver === scripted data driver: launch then full throttle straight ahead
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 1.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 1.0, 0.7, 0.0),
])
driver = veh.ChDataDriver(car.GetVehicle(), driver_data)
driver.Initialize()

# === Main loop === real-time render-throttled Synchronize/Advance stack
render_steps = math.ceil(render_step_size / step_size)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)             # match wall clock to sim time
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
