"""ARTcar rigid-terrain vehicle demo using NSC contact.

The simulation places an ARTcar chassis at (1, 0, 0.5), uses primitive
visualization for the vehicle parts, enables mesh chassis collision, and uses
FIALA tires on a flat rigid terrain.  The expected behavior is a stable vehicle
scene that can be driven across the terrain while reporting the vehicle mass and
basic motion data during review runs.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === physics and rendering values are named for review clarity
TIME_STEP = 1.0e-3
TIRE_STEP_SIZE = TIME_STEP
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 20.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(1.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
VIS_TYPE = veh.VisualizationType_PRIMITIVES


# === Vehicle setup === wrapper owns the ChSystem and all vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_MESH)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_FIALA)  # prompt: FIALA tire model
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: main chassis rigid body reused for logging
veh_obj = vehicle.GetVehicle()  # cache: wrapper vehicle handle reused for mass/spindles
# The ARTcar wrapper creates chassis, suspension, steering, wheel, tire, and joint bodies.
print("VEHICLE MASS: ", veh_obj.GetMass())

vehicle.SetChassisVisualizationType(VIS_TYPE)
vehicle.SetSuspensionVisualizationType(VIS_TYPE)
vehicle.SetSteeringVisualizationType(VIS_TYPE)
vehicle.SetWheelVisualizationType(VIS_TYPE)
vehicle.SetTireVisualizationType(VIS_TYPE)

spindle_positions = []  # cache: initial support check avoids repeated getter calls
for axle_index in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
assert min(p.z for p in spindle_positions) > 0.0, "ARTcar wheels must initialize above terrain"


# === Terrain === flat rigid support uses NSC material matching the wrapper contact method
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 10)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization === vehicle-aware Irrlicht scene is built unconditionally
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar FIALA Tire Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.0), 7.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)


# === Driver === interactive driver matches catalog vehicle demo structure
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Main loop === synchronize and advance the full vehicle subsystem stack
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()


            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(TIME_STEP)
except (RuntimeError, ValueError) as exc:  # guard Chrono runtime failures and invalid state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # guard review output file-system errors
    traceback.print_exc()
    raise
finally:
    pass
