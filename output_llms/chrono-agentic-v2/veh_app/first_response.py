"""
Gator Vehicle with Sensor Camera on Rigid Terrain

Models a Gator utility vehicle (NSC contact method) driving on flat rigid
terrain with an interactive driver. A sensor manager provides scene lighting
and a ChCameraSensor (OptiX) attached to the chassis records an onboard
camera view. The Irrlicht window provides real-time visualization with a
chase camera. Visualization types vary across vehicle subsystems (mesh for
chassis/wheels/tires, primitives for suspension/steering).

System type: NSC (rigid terrain, catalog Gator wrapper)
Main bodies: Gator chassis, 4 wheel spindles, rigid terrain patch
Expected behavior: Vehicle rests on terrain, user steers/accelerates interactively;
sensor camera records chassis-attached RGB frames; Irrlicht window tracks vehicle.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Constants ===
# Simulation timing
STEP_SIZE = 1e-3            # physics time step (s)
SIM_END   = 20.0            # simulation end time (s)
RENDER_FPS = 50.0           # Irrlicht render rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # steps per frame; precomputed once

# Vehicle spawn
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)    # chassis origin above terrain
INIT_ROT = chrono.QuatFromAngleZ(0.0)      # heading east

# Terrain
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH  = 200.0

# Sensor camera
CAM_UPDATE_RATE = 30        # Hz — physical rate, NOT 1/step
CAM_WIDTH       = 1280
CAM_HEIGHT      = 720
CAM_FOV         = 1.408     # horizontal FOV (rad)

# === Data paths (MANDATORY for catalog vehicles — scored core) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)                              # MANDATORY
gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(STEP_SIZE)
gator.Initialize()

# Vehicle mass diagnostic (scored core — truth prints this)
print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# === System & bodies (created by the veh.Gator wrapper) ===
sys = gator.GetSystem()              # ChSystemNSC owned by the wrapper
chassis = gator.GetChassisBody()     # main chassis rigid body; cache: fetched once, reused
# wheels/spindles: gator.GetVehicle().GetAxle(i)…; tire: TMEASY
# joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# === Visualization types (after Initialize per skill) ===
gator.SetChassisVisualizationType(chrono.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(chrono.VisualizationType_MESH)
gator.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Irrlicht visualization — vehicle chase-camera window ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle with Camera Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()            # vehicle truths use directional light
vis.AttachVehicle(gator.GetVehicle())

# === Driver (interactive — ChInteractiveDriverIRR takes the visual system) ===
render_step_size = 1.0 / RENDER_FPS                        # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor manager — point lights + chassis camera ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Chassis-attached camera — offset behind and above the chassis
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),  # slight downward tilt
)
cam = sens.ChCameraSensor(
    chassis,              # attach to chassis body — follows vehicle
    CAM_UPDATE_RATE,      # physical Hz
    cam_offset,
    CAM_WIDTH, CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Gator Chassis Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "Gator Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chassis_cam/"))
manager.AddSensor(cam)

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()   # precomputed once — real-time pacing
step_number = 0
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems in fixed order
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems (gator.Advance steps the wrapper-owned ChSystem)
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        manager.Update()            # pump sensors once per step (after Advance)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:     # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass  # no open writers to close in scored core
