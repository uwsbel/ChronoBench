"""Curiosity rover navigating on rigid terrain (PyChrono 9.0.1 + Irrlicht).

Models the NASA Curiosity (MSL) six-wheel rover from `pychrono.robot` driving on
a fixed rigid ground patch. System type: ChSystemNSC (rigid, non-smooth contact),
Bullet narrow-phase collision. Main bodies: the rover (chassis + rocker-bogie
suspension + six wheels, created by robot.Curiosity) and a fixed collision-enabled
ground box. The rover is driven by a CuriosityDCMotorControl driver that provides
real-time wheel-speed and steering commands; a gentle time-varying steering input
makes the rover follow a curving path. Expected behavior: the rover accelerates
forward from rest and traverses the terrain, steering left/right, with its wheels
in rolling contact on the ground.

Visualization: Irrlicht window with an elevated side camera that frames the full
traverse against a fixed ground grid, plus sky box, logo, typical lights with a
shadow-casting key light, a ground reference grid, and a textured ground patch.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot

# === Parameters === geometry / physics constants (no bare literals downstream)
TIME_STEP = 2.0e-3            # solver step [s]
SIM_END = 12.0               # total simulated time [s]
RENDER_FPS = 25.0            # review render cadence [frames/s]

GROUND_SX = 40.0             # ground patch length [m]
GROUND_SY = 40.0             # ground patch width  [m]
GROUND_SZ = 1.0              # ground patch thickness [m]
GROUND_TOP_Z = 0.0           # top surface of the ground [m]
GROUND_FRICTION = 0.9        # tire/soil friction coefficient
GROUND_RESTITUTION = 0.0     # fully inelastic ground contact

ROVER_SPAWN_Z = GROUND_TOP_Z + 0.2   # suspension reference height above terrain [m]
MOTOR_NO_LOAD_SPEED = math.pi        # wheel motor no-load speed [rad/s]
MOTOR_STALL_TORQUE = 300.0           # wheel motor stall torque [N*m]
STEER_AMPLITUDE = 0.30               # peak steering angle [rad]
STEER_PERIOD = 8.0                   # steering oscillation period [s]

# Fixed elevated side camera, framing the ~8 m traverse against the static grid.
CAM_EYE = chrono.ChVector3d(4.0, -10.0, 5.0)     # camera position [m]
CAM_TARGET = chrono.ChVector3d(4.0, -1.0, 0.3)   # look-at near the path midpoint [m]

# Derived constants — precomputed once (never recomputed in the hot loop)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
STEER_OMEGA = 2.0 * math.pi / STEER_PERIOD                     # precomputed once
SPAWN_POS = chrono.ChVector3d(0.0, 0.0, ROVER_SPAWN_Z)         # precomputed once
SPAWN_ROT = chrono.QuatFromAngleZ(0.0)                         # precomputed once

# === System & gravity === NSC rigid system + Bullet collision for rover/ground contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required: rover-ground contact

# === Contact material === rigid-terrain NSC material (matches ChSystemNSC)
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(GROUND_RESTITUTION)

# === Bodies === fixed, collision-enabled, textured ground patch
ground = chrono.ChBodyEasyBox(GROUND_SX, GROUND_SY, GROUND_SZ, 1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_TOP_Z - 0.5 * GROUND_SZ))  # top surface at GROUND_TOP_Z
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
sys.Add(ground)

# === Rover === Curiosity (chassis + rocker-bogie suspension + six wheels) with DC motor driver
driver = robot.CuriosityDCMotorControl()
driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_LF)
driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_RF)
driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_LM)
driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_RM)
driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_LB)
driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, robot.C_RB)
driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_LF)
driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_RF)
driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_LM)
driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_RM)
driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_LB)
driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, robot.C_RB)

rover = robot.Curiosity(sys)
rover.SetDriver(driver)
rover.SetWheelContactMaterial(ground_mat)
rover.Initialize(chrono.ChFramed(SPAWN_POS, SPAWN_ROT))

chassis = rover.GetChassis()                  # cache: chassis part, reused every step
# Assert the rover starts on (not through) the ground rather than trusting a comment.
assert rover.GetChassisPos().z > GROUND_TOP_Z, "rover spawned below the terrain top"

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # gravity along -Z -> Z-up camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover on Rigid Terrain")
vis.Initialize()                                    # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                                     # standard outdoor sky backdrop
vis.AddCamera(CAM_EYE, CAM_TARGET)                  # fixed side view of the full traverse
vis.AddTypicalLights()                              # standard two-light setup
vis.AddLightWithShadow(chrono.ChVector3d(20.0, 35.0, 25.0),
                       chrono.ChVector3d(0, 0, 0),
                       55, 20, 80, 35, 512)         # shadow-casting key light
vis.EnableShadows()
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z + 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

# === Main loop === render-cadence outer loop; physics advanced in inner batches


frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()
            steer = STEER_AMPLITUDE * math.sin(STEER_OMEGA * t)
            driver.SetSteering(steer)
            rover.Update()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise
