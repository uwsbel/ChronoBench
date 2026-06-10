"""Full HMMWV on SCM deformable terrain using an SMC vehicle system.

The script builds a catalog HMMWV with mesh visualization and rigid tires, places it
on Bekker-Wong SCM soil with a chassis-following moving patch, colors terrain
sinkage, and runs a real-time Irrlicht interactive-driver simulation.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters keep the SCM vehicle setup auditable
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 40.0
TERRAIN_DELTA = 0.08
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.6)
INIT_ROT = chrono.QUNIT
MOVING_PATCH_DIMS = chrono.ChVector3d(5.0, 3.0, 1.0)


# === Vehicle === wrapper owns the SMC system and creates chassis, suspension, wheels, and joints
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_RIGID)  # prompt: rigid tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemSMC reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: chassis body anchors SCM moving patch
vehicle_obj = hmmwv.GetVehicle()  # cache: low-level vehicle handle for mass/spindles
print("VEHICLE MASS: ", vehicle_obj.GetMass())

# wrapper-created bodies: chassis, four suspensions, steering, wheel/spindle bodies, tires
# wrapper-created joints: suspension links, steering links, driveline constraints
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

spindle_positions = []
for axle_index in range(vehicle_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle_obj.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - 0.47
assert wheel_bottom_z >= -0.08, (
    f"vehicle starts too deep in SCM: wheel bottom z={wheel_bottom_z:.3f}"
)


# === Terrain === SCM soil deforms under the vehicle and follows the chassis efficiently
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2.0e6,
    0.0,
    1.1,
    0.0,
    30.0,
    0.01,
    2.0e8,
    3.0e4,
)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0.0, 0.0, 0.0), MOVING_PATCH_DIMS)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.10)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 40)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_DELTA)


# === Visualization === vehicle-aware Irrlicht window with chase camera and directional light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_obj)


# === Driver === interactive controls match the real-time vehicle demo pattern
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()


# === Main loop === synchronize driver, SCM terrain, HMMWV, and Irrlicht at each step
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()  # cache: shared by driver, vehicle, vis, and logging

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # guard review file/image writes and filesystem errors
    traceback.print_exc()
    raise
finally:
    pass
