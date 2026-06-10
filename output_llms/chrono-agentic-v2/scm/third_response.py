"""
HMMWV on SCM Deformable Terrain with Randomly Positioned Boxes and Camera Sensor.

System type: ChSystemSMC (owned by HMMWV_Full wrapper via SMC contact method for SCM).
Main bodies: HMMWV chassis, 4 wheel spindles, SCM deformable terrain, randomly-placed
ChBodyEasyBox obstacles scattered around the scene (avoiding the vehicle spawn zone).
A ChSensorManager with point lights and a ChCameraSensor attached to the chassis
(with ChFilterVisualize) provides live camera feed visualization.

Expected behavior: HMMWV drives forward on soft SCM terrain, leaving visible ruts;
boxes are scattered across the terrain surface; the camera sensor shows a chase view
from behind the chassis with a live preview window.
"""

import math
import os
import random
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Data paths (vehicle truth mandatory) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
STEP_SIZE         = 2e-3       # physics time step (s)
SIM_END           = 15.0       # simulation duration (s)
RENDER_FPS        = 50.0
render_every      = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH    = 120.0      # SCM patch length (m)
TERRAIN_WIDTH     = 120.0      # SCM patch width (m)
TERRAIN_RES       = 0.1        # SCM grid resolution (m)

# Vehicle initial position
VEH_INIT_X        = 0.0
VEH_INIT_Y        = 0.0
SUSPENSION_REF_H  = 0.5        # chassis origin above wheel-bottom at rest (m)
VEH_INIT_Z        = SUSPENSION_REF_H

# Tire geometry (TMEASY on HMMWV)
TIRE_RADIUS       = 0.33
TIRE_WIDTH        = 0.22

TIRE_FAMILY       = 1
SUPPORT_FAMILY    = 4
CHASSIS_FAMILY    = 3

# Random box parameters
NUM_BOXES         = 20
BOX_HALF_X        = 0.4        # half-size x
BOX_HALF_Y        = 0.3        # half-size y
BOX_HALF_Z        = 0.3        # half-size z
BOX_MASS_DENSITY  = 500.0      # kg/m^3
EXCLUSION_RADIUS  = 5.0        # boxes spawned outside this radius of vehicle start

# Camera sensor settings
CAM_UPDATE_RATE   = 30         # Hz (physical, not 1/dt)
CAM_WIDTH         = 1280
CAM_HEIGHT        = 720
CAM_FOV           = 1.408      # horizontal FOV (rad)

# Camera offset: behind and above the chassis
CAM_OFFSET_X      = -8.0
CAM_OFFSET_Z      = 2.5
CAM_TILT_ANGLE    = 0.15       # rad tilt toward chassis center

RNG_SEED          = 42

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SCM requires SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
    chrono.QUNIT,
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # SCM requires non-rigid tire
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: SCMTerrain below
# joints: suspension + steering created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === SCM Terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi
    0,      # Bekker_Kc
    1.1,    # Bekker_n
    0,      # Mohr_cohesion
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K (Pa/m)
    3e4,    # damping_R (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.AddMovingPatch(
    chassis,                              # CORRECT: chassis stays level
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

# === Tire collision cylinders (required for TMEASY on SCM) ===
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, TIRE_RADIUS + 0.04, TIRE_WIDTH),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()  # rebuild all collision models after shape changes

# === Randomly positioned boxes ===
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.0)
box_mat.SetYoungModulus(2e7)

rng = random.Random(RNG_SEED)
box_bodies = []   # cache: list of box bodies added to system
for _ in range(NUM_BOXES):
    # Sample a position far enough from vehicle spawn
    while True:
        bx = rng.uniform(-TERRAIN_LENGTH / 2 * 0.8, TERRAIN_LENGTH / 2 * 0.8)
        by = rng.uniform(-TERRAIN_WIDTH  / 2 * 0.8, TERRAIN_WIDTH  / 2 * 0.8)
        dist = math.sqrt((bx - VEH_INIT_X) ** 2 + (by - VEH_INIT_Y) ** 2)
        if dist > EXCLUSION_RADIUS:
            break
    bz = BOX_HALF_Z  # rest on z=0 (SCM rest plane)
    box = chrono.ChBodyEasyBox(
        BOX_HALF_X * 2, BOX_HALF_Y * 2, BOX_HALF_Z * 2,
        BOX_MASS_DENSITY, True, True, box_mat,
    )
    box.SetPos(chrono.ChVector3d(bx, by, bz))
    box.SetFixed(False)
    system.AddBody(box)
    box_bodies.append(box)

# === Sensor manager with point lights ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(20, 20, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-20, -20, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor attached to chassis ===
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(CAM_OFFSET_X, 0, CAM_OFFSET_Z),
    chrono.QuatFromAngleAxis(CAM_TILT_ANGLE, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis,            # attach to the real chassis body
    CAM_UPDATE_RATE,    # physical update rate (Hz)
    cam_offset_pose,
    CAM_WIDTH, CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Chassis Camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "Chase Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/chase/"))
manager.AddSensor(cam)

# === Interactive driver (scored-core default for catalog vehicles) ===
# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM with Boxes and Camera Sensor")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()          # vehicle truths use directional, not typical lights
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(1.0 / (RENDER_FPS * steering_time))
driver.SetThrottleDelta(1.0 / (RENDER_FPS * throttle_time))
driver.SetBrakingDelta(1.0 / (RENDER_FPS * braking_time))
driver.Initialize()

# === Review-only setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        manager.Update()   # pump sensors every physics step


        step_number += 1
        if system.GetChTime() >= SIM_END:
            break
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
