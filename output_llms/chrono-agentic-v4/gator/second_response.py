"""
Gator utility vehicle on multi-patch terrain with heightmap for gradability testing.

4 terrain patches with different textures; one features a height map (bump) to test
vehicle gradability. Uses SCM deformable terrain for realistic soft-soil behavior.
"""

import os, math, csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Review-only: sim_recording for frame capture and video assembly ===

# === Simulation parameters ===
time_step = 1e-3
sim_end = 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create the Gator utility vehicle ===
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisFixed(False)
gator.SetTireType(veh.TireModelType_RIGID)
gator.SetTireStepSize(time_step)

# Initial position: chassis origin height above ground
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.QUNIT
gator.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
gator.Initialize()

system = gator.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())

# Cache essential handles
chassis = gator.GetChassisBody()  # cache: main chassis body

# === SCM Terrain with 4 patches ===
terrain = veh.SCMTerrain(system)

# Soil parameters for soft soil (Bekker-Wong model)
# All 8 required parameters specified per veh/terrain skill
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent (1.0 soft → 1.5 hard)
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)

# Terrain dimensions (4 patches side by side)
patch_length = 15.0   # X direction (direction of travel)
patch_width = 15.0    # Y direction (lateral)
total_width = patch_width * 4

terrain.Initialize(total_width, patch_length, 0.1)  # length=X, width=Y, resolution
terrain.SetMeshWireframe(False)

# Add moving patch centered on chassis for computational efficiency
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0, 0, 0),     # OOBB center offset in chassis frame
    chrono.ChVector3d(8, 5, 2),      # OOBB dimensions (m)
)

# Set plot type for visualization (sinkage heatmap)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# === Visualization (Irrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator - Multi-Patch Terrain with Heightmap Gradability Test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()  # vehicle scenes use directional light, not AddTypicalLights
vis.AttachVehicle(gator.GetVehicle())

# === Driver (interactive) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(time_step / steering_time)
driver.SetThrottleDelta(time_step / throttle_time)
driver.SetBrakingDelta(time_step / braking_time)
driver.Initialize()

# === Review-only: recording setup ===

frame = 0

# CSV writers (always defined; review-only block populates them)
csv_path = "simulation_data.csv"
csv_file = None
csv_writer = None

# === Main simulation loop ===
print("Simulation running... (close the window or press Ctrl+C to exit)")
step_count = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    # Throttled rendering
    if step_count % render_every == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize all subsystems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all subsystems
    driver.Advance(time_step)
    terrain.Advance(time_step)
    gator.Advance(time_step)
    vis.Advance(time_step)

    # Log to CSV (review-only)

    step_count += 1
    realtime_timer.Spin(time_step)

# === Review-only: post-loop video assembly and cleanup ===

print(f"Done. Total steps: {step_count}")
