"""Full HMMWV on SCM deformable terrain (Bekker-Wong soft soil), Irrlicht render.

Models a full HMMWV (veh.HMMWV_Full) driving on an SCMTerrain patch with custom
soil parameters and a moving patch that follows the chassis. The vehicle uses a
RIGID tire model and mesh visualization on all components. The terrain renders
sinkage with a false-color (PLOT_SINKAGE) overlay. An interactive driver
(ChInteractiveDriverIRR) controls steering / throttle / braking, and the sim runs
in real time, rendering at 50 fps.

System type: NSC contact via the HMMWV_Full wrapper (its own ChSystem), Bullet
collision. Main bodies: HMMWV chassis + four wheels/spindles, the deformable SCM
terrain grid. Expected behavior: wheels press into the soft soil leaving visible
ruts/sinkage as the vehicle is driven.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics / soil constants (no bare literals downstream)
time_step = 2e-3                      # integration step (s)
sim_end = 12.0                        # bounded recording horizon (s)
render_fps = 50.0                     # render cadence requested by the task
render_step_size = 1.0 / render_fps   # precomputed once: seconds between frames
render_steps = math.ceil(render_step_size / time_step)  # precomputed once: steps/frame

terrain_length = 60.0                 # SCM patch X extent (m)
terrain_width = 60.0                  # SCM patch Y extent (m)
terrain_res = 0.1                     # SCM grid resolution (m)
init_loc = chrono.ChVector3d(-20.0, 0.0, 0.6)   # chassis spawn (origin = geom center)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)     # identity orientation

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV with rigid tires on deformable soil
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)   # SCM/deformable scenes use SMC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_RIGID)           # prompt: rigid tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

# Mesh visualization on ALL vehicle components (prompt-required)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = vehicle.GetSystem()                           # ChSystemSMC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = vehicle.GetChassisBody()  # cache: main chassis rigid body, reused for moving patch
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())   # report total vehicle mass
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); joints: suspension + steering inside wrapper

# === Terrain === SCM deformable soil with custom parameters + chassis-following patch
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — sinkage exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa.s/m)
)
# Moving patch on the CHASSIS (level body -> stable OOBB; spindles would give rays=0)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
# False-color sinkage plot (prompt-required) — set before Initialize
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(terrain_length, terrain_width, terrain_res)
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

# === Visualization === vehicle-aware Irrlicht: window + sky + chase cam + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                              # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive steering / throttle / braking bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)        # s to reach full steering
driver.SetThrottleDelta(render_step_size / 1.0)        # s to reach full throttle
driver.SetBrakingDelta(render_step_size / 0.3)         # s to reach full braking
driver.Initialize()

# === Main loop === real-time Synchronize/Advance stack, rendering at 50 fps

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:            # throttled rendering at render_fps
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)                     # advances the wrapper-owned system
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)                 # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:              # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
