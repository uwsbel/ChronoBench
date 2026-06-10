"""HMMWV rigid-highway simulation using an NSC vehicle system and Irrlicht.

The script builds a full HMMWV with TMEASY tires on a custom rigid mesh terrain:
Highway_col.obj provides collision and Highway_vis.obj provides the visible road.
An interactive Irrlicht driver controls steering, throttle, and braking while the
vehicle and terrain modules synchronize in a real-time 50 FPS render loop.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters keep vehicle, terrain, and timing explicit
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

INIT_LOC = chrono.ChVector3d(6.0, -70.0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(1.57)
TRACK_POINT = chrono.ChVector3d(-3.0, 0.0, 1.1)
CONTACT_METHOD = chrono.ChContactMethod_NSC
VIS_TYPE = veh.VisualizationType_MESH
CHASSIS_COLLISION_TYPE = veh.CollisionType_NONE
TIRE_MODEL = veh.TireModelType_TMEASY
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_STEP_SIZE = 1.0 / 50.0
RENDER_STEPS = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)  # precomputed once


# === Vehicle === full HMMWV wrapper owns the Chrono system and vehicle bodies
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(CONTACT_METHOD)
vehicle.SetChassisCollisionType(CHASSIS_COLLISION_TYPE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(TIRE_MODEL)  # prompt: TMEASY tire model
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(VIS_TYPE)
vehicle.SetSuspensionVisualizationType(VIS_TYPE)
vehicle.SetSteeringVisualizationType(VIS_TYPE)
vehicle.SetWheelVisualizationType(VIS_TYPE)
vehicle.SetTireVisualizationType(VIS_TYPE)

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: main chassis body for logging
veh_obj = vehicle.GetVehicle()  # cache: vehicle assembly handle for mass/spindles
# wrapper-created components: system, chassis, suspension/steering joints, wheels,
# TMEASY tires, powertrain, and axles are created inside veh.HMMWV_Full.
print("VEHICLE MASS: ", veh_obj.GetMass())

spindle_positions = []
for axle_index in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - 0.47
assert wheel_bottom_z >= -0.08, (
    f"vehicle sinks into highway mesh: wheel bottom z={wheel_bottom_z:.3f}; "
    "raise INIT_LOC.z"
)


# === Terrain === rigid mesh road uses collision OBJ plus separate visual OBJ
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj"),
    True,
    0.01,
    False,
)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True
)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)
terrain.Initialize()


# === Visualization and driver === Irrlicht window and interactive driver match vehicle demos
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Rigid Highway")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(TRACK_POINT, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(veh_obj)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()
run_driver = driver


# === Main loop === synchronize driver, terrain, vehicle, and visualization every step
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = run_driver.GetInputs()


        run_driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        run_driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (OSError, IOError) as exc:  # disk or permission failure for output files
    traceback.print_exc()
    raise
except (RuntimeError, ValueError, AssertionError) as exc:  # Chrono runtime or bad state guard
    traceback.print_exc()
    raise
finally:
    pass
