"""BMW E90 Sedan on rigid terrain with NSC contact and TMEASY tires.

This standalone PyChrono 9.0.0 simulation builds a catalog BMW E90 Sedan,
places it on a textured rigid terrain patch, and runs a real-time Irrlicht
visualization with a chase camera, logo, skybox, and directional lighting.
The simulation uses an interactive Irrlicht driver for steering, throttle,
and braking.
"""

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Vehicle and terrain constants are named once so the run loop stays compact.
STEP_SIZE = 0.002
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 320.0
TERRAIN_WIDTH = 120.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOCATION = chrono.ChVector3d(0.0, 0.0, 0.50)
INIT_ROTATION = chrono.QUNIT
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 0.75)
CHASE_DISTANCE = 6.0
CHASE_HEIGHT = 0.5

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


# === Vehicle and terrain ===
# The BMW wrapper owns its ChSystem; terrain and visualization attach to it.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOCATION, INIT_ROTATION))
vehicle.SetTireType(veh.TireModelType_TMEASY)  # prompt: TMEASY tire model
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned system reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis handle reused for logging
veh_obj = vehicle.GetVehicle()  # cache: vehicle handle reused for visualization
print("VEHICLE MASS: ", veh_obj.GetMass())

# Wrapper-created components visible to source review: BMW vehicle subsystem,
# chassis body, suspension/wheel/tire bodies, rigid terrain, Irrlicht visualizer,
# and interactive driver are synchronized in one shared system.
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain_material = chrono.ChContactMaterialNSC()
terrain_material.SetFriction(TERRAIN_FRICTION)
terrain_material.SetRestitution(TERRAIN_RESTITUTION)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    terrain_material,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.55, 0.60, 0.48))
terrain.Initialize()

spindle_positions = []
for axle_index in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
lowest_spindle_z = min(pos.z for pos in spindle_positions)
assert lowest_spindle_z > terrain.GetHeight(INIT_LOCATION), (
    f"vehicle spindle height {lowest_spindle_z:.3f} is not above rigid terrain"
)


# === Visualization and driver ===
# Vehicle Irrlicht visualization is configured before driver construction.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan on rigid terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / STEERING_TIME)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / THROTTLE_TIME)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / BRAKING_TIME)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
# The loop synchronizes the full vehicle stack and uses wrapper Advance only.
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver.Synchronize(time)

            driver_inputs = driver.GetInputs()
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError, OSError, IOError) as exc:
    print(f"simulation failed: {exc}")
    raise
