"""UAZBUS wheeled-vehicle mobility test on flat rigid terrain (PyChrono 9.0.x).

Models a UAZBUS catalog wheeled vehicle (NSC contact system, owned by the
veh.UAZBUS wrapper) driving forward on a flat RigidTerrain patch toward a fixed
box obstacle placed in its path. The vehicle uses a RIGID tire model and is
driven open-loop with a constant 0.5 throttle and zero steering, so it
accelerates straight ahead (+X) and interacts with the obstacle. Visualization
is the vehicle-aware Irrlicht chase-camera window.

Expected behavior: the bus starts at rest on the terrain, builds forward speed
under constant throttle, and drives along +X until it reaches/contacts the fixed
box obstacle at x = 5 m, testing its mobility against the obstruction.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation constants === time stepping, run length, render cadence
time_step = 1e-3                    # integration step (s)
sim_end = 8.0                       # total simulated time (s)
render_fps = 50.0                   # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once: steps/frame

# === Vehicle placement constants === spawn pose derived from tire geometry
TIRE_RADIUS = 0.372                 # UAZBUS tire radius (m), from wheel geometry
SUSPENSION_REF_HEIGHT = 0.45        # chassis origin height above terrain at rest (m)
TERRAIN_TOP_Z = 0.0                 # flat rigid terrain top plane (m)
VEH_INIT_X = 0.0                    # spawn X (m)
VEH_INIT_Y = 0.0                    # spawn Y (m)
init_z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT          # derived chassis Z
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, init_z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)             # identity: facing +X

# === Terrain extent / obstacle constants ===
TERRAIN_LENGTH = 100.0              # rigid patch X size (m)
TERRAIN_WIDTH = 100.0               # rigid patch Y size (m)
BOX_SIZE = chrono.ChVector3d(0.5, 5.0, 0.2)             # obstacle full extents (m)
BOX_POS = chrono.ChVector3d(5.0, 0.0, 0.1)              # obstacle center (m)

# === Driver control constants === open-loop straight-line drive
DRIVE_THROTTLE = 0.5                # constant throttle command (0..1)
DRIVE_STEERING = 0.0               # straight ahead

# === Vehicle (created and owned by the veh.UAZBUS wrapper) ===
# The wrapper internally builds the ChSystemNSC, the chassis rigid body, the
# four spindles/wheels, and the suspension + steering joints.
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_RIGID)            # prompt: rigid tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (handles fetched from the wrapper) ===
sys = vehicle.GetSystem()                 # ChSystemNSC owned by the UAZBUS wrapper
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused every step
veh_obj = vehicle.GetVehicle()            # cache: ChWheeledVehicle handle for spindle/state queries

# Collision must use Bullet for vehicle/terrain/obstacle contact.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Verify the wheels rest on (not through) the terrain after Initialize.
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - 0.05, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch under the vehicle (Bullet contacts)
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Obstacle === fixed box in the vehicle's path to test mobility
obstacle_mat = chrono.ChContactMaterialNSC()
obstacle_mat.SetFriction(0.9)
obstacle_mat.SetRestitution(0.0)
obstacle = chrono.ChBodyEasyBox(
    BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000.0, True, True, obstacle_mat
)
obstacle.SetPos(BOX_POS)
obstacle.SetFixed(True)                   # anchored obstruction
obstacle.SetName("box_obstacle")
sys.AddBody(obstacle)

# === Driver === open-loop constant throttle, straight ahead
class ConstantDriver(veh.ChDriver):
    """Scripted driver applying a constant forward throttle and no steering."""

    def __init__(self, vehicle_handle):
        super().__init__(vehicle_handle)

    def Synchronize(self, time):
        self.SetThrottle(DRIVE_THROTTLE)
        self.SetSteering(DRIVE_STEERING)
        self.SetBraking(0.0)

driver = ConstantDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht chase-camera window
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS mobility test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 60, 60,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.35, 0.35, 0.35))   # ground reference grid
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === render-cadence outer loop; Synchronize/Advance per step
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(time_step)
            terrain.Advance(time_step)
            vehicle.Advance(time_step)        # advances the wrapper-owned system
            vis.Advance(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
