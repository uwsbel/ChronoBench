"""
Viper Rover on SCM Deformable Terrain — PyChrono 9.0.x / Irrlicht

Models a Viper six-wheel rover traversing Bekker-Wong soft-soil terrain
(SCMTerrain). The rover drives straight ahead (constant steering = 0.0)
while the terrain deforms under the wheels leaving visible ruts.

System: ChSystemNSC + Bullet collision
Terrain: veh.SCMTerrain (deformable, Bekker-Wong soil model)
Robot: robot.Viper with ViperDCMotorControl, steering held at 0.0
Expected behavior: rover rolls forward in a straight line, sinking into
the SCM surface and leaving symmetric wheel ruts.
"""

# === Imports ===
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step   = 1e-3      # physics time step (s)
sim_end     = 15.0      # total simulation duration (s)
render_fps  = 50.0      # frames per second for review video
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per render frame

# SCM terrain parameters
terrain_length   = 14.0   # X extent (m)
terrain_width    =  4.0   # Y extent (m)
terrain_delta    =  0.02  # mesh resolution (m) — fine enough to show ruts

# Rover spawn above SCM rest plane (z = 0)
rover_init_pos = chrono.ChVector3d(0, 0, 0.2)
rover_init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (w,x,y,z), facing +X

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === SCM Deformable Terrain ===
# veh.SCMTerrain requires the collision system to be set before construction
terrain = veh.SCMTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5)))  # rest plane at z = -0.5

terrain.Initialize(terrain_length, terrain_width, terrain_delta)

# Bekker-Wong soil parameters (8 positional args required)
terrain.SetSoilParameters(
    0.2e6,   # Bekker_Kphi    — frictional modulus (Pa)
    0,       # Bekker_Kc      — cohesive modulus
    1.1,     # Bekker_n       — exponent
    0,       # Mohr_cohesion  — cohesive limit (Pa)
    30,      # Mohr_friction  — friction angle (deg)
    0.01,    # Janosi_shear   — shear coefficient (m)
    4e7,     # elastic_K      — elastic stiffness (Pa/m)
    3e4,     # damping_R      — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)  # pressure heatmap overlay
terrain.SetMeshWireframe(True)

# === Rover ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(rover_init_pos, rover_init_rot))

# Cache chassis body — attach moving patch + sensors here
chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused each step

# Attach a moving patch to the chassis so only cells near the rover are updated
terrain.AddMovingPatch(
    chassis_body,
    chrono.ChVector3d(0, 0, 0),    # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),    # OOBB dimensions (m)
)

# === Visualization (Irrlicht — Initialize FIRST, scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-4, 3, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()  # cache: current sim time, used multiple times this iter

        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        for _ in range(render_every):
            time = system.GetChTime()

            # Constant steering = 0.0 (drive straight)
            driver.SetSteering(0.0)
            rover.Update()


            system.DoStepDynamics(time_step)

            if system.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass  # CSV closed in review-only block below
