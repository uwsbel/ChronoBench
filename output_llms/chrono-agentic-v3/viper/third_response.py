"""
Viper rover on SCM deformable terrain (PyChrono 9.0.x, Irrlicht, ChSystemNSC).

Models the Viper 6-wheel rover driving straight on a Bekker-Wong soft-soil
SCM terrain. The rover uses a ViperDCMotorControl driver with constant
steering = 0.0 (drives straight). The SCM terrain deforms under the wheels,
producing visible sinkage and ruts. Gravity is -Z.

System: ChSystemNSC + Bullet collision
Bodies: Viper rover (chassis + 6 wheels/suspension, owned by robot.Viper),
        SCM deformable terrain surface
Expected behavior: rover rolls forward in a straight line, wheels sink into
                   soft soil, leaving ruts behind it.
"""

# === Imports ===
import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
TIME_STEP = 1e-3          # physics step (s)
SIM_END   = 10.0          # simulation duration (s)
RENDER_FPS = 50.0         # Irrlicht frame rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Rover spawn
ROVER_INIT_POS = chrono.ChVector3d(0, 0, 0.2)
ROVER_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity

# SCM terrain
SCM_SIZE_X  = 14.0    # terrain X extent (m)
SCM_SIZE_Y  = 4.0     # terrain Y extent (m)
SCM_DELTA   = 0.02    # grid resolution (m) — fine enough for visible ruts
SCM_PLANE_Z = -0.5    # terrain plane reference height (z)

# === System & collision ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === SCM deformable terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, SCM_PLANE_Z)))
terrain.Initialize(SCM_SIZE_X, SCM_SIZE_Y, SCM_DELTA)
terrain.SetSoilParameters(
    0.2e6,  # Bekker_Kphi    frictional modulus (Pa)
    0,      # Bekker_Kc      cohesive modulus
    1.1,    # Bekker_n       exponent
    0,      # Mohr_cohesion  cohesive limit (Pa)
    30,     # Mohr_friction  friction angle (deg)
    0.01,   # Janosi_shear   shear coefficient (m)
    4e7,    # elastic_K      elastic stiffness (Pa/m)
    3e4,    # damping_R      vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)
terrain.SetMeshWireframe(True)

# === Rover ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

chassis_body = rover.GetChassis().GetBody()   # cache: reused every step

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Review-only setup ===


# === Main loop ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: fetched once per outer frame

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            # Constant steering = 0.0 (drive straight)
            driver.SetSteering(0.0)
            rover.Update()


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
