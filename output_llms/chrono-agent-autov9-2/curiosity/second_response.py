"""Curiosity Mars rover driving straight across a long box obstacle.

Model:
  - System: ChSystemNSC (rigid-body contact dynamics), Z-up world, gravity -Z.
  - Bodies: a fixed rigid ground plane, a long fixed box obstacle the rover must
    climb over, and the six-wheel rocker-bogie Curiosity rover (robot.Curiosity).
  - Driver: CuriosityDCMotorControl (DC-motor speed control) with ZERO steering,
    so the rover drives straight forward along +X.
Expected behavior:
  The rover spawns at (-5, 0, 0) on the ground, drives forward in a straight line,
  reaches the low long box obstacle, and the rocker-bogie suspension lets it climb
  over and continue past it. Contact between wheels/ground/obstacle uses the Bullet
  collision system.
"""

import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics (no bare position literals downstream)
TIME_STEP = 2e-3                       # contact-rich rover dynamics
SIM_END = 14.0                         # long enough to reach and cross the obstacle
RENDER_FPS = 25.0                      # modest cadence so render stays under timeout

GROUND_SIZE = 30.0                     # square ground plane edge length (m)
GROUND_THICK = 1.0                     # ground slab thickness (m)

ROVER_START = chrono.ChVector3d(-5.0, 0.0, 0.0)   # rover initial position (prompt)

# Long box obstacle the rover crosses: long across the path (Y), low so the
# rocker-bogie can climb it, placed ahead of the rover along +X.
OBST_LEN_X = 0.8                       # extent along travel direction (m)
OBST_LEN_Y = 6.0                       # "long" extent across the path (m)
OBST_HEIGHT = 0.10                     # low ridge the rocker-bogie reliably climbs (m)
OBST_CENTER = chrono.ChVector3d(0.0, 0.0, OBST_HEIGHT / 2.0)  # rests on ground

MOTOR_NO_LOAD_SPEED = math.pi          # rad/s wheel free speed -> steady forward drive
MOTOR_STALL_TORQUE = 300.0             # N*m, ample to climb the low obstacle

# === System & gravity === rigid contact dynamics, Z-up
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Bullet narrow-phase collision is REQUIRED: ground + obstacle + rover wheels touch.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material === shared ground/obstacle friction for the wheels
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)            # high friction so wheels grip and climb
ground_mat.SetRestitution(0.0)

# === Bodies === ground plane + long box obstacle (both fixed)
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                              1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -GROUND_THICK / 2.0))   # top surface at z=0
ground.SetFixed(True)
sys.Add(ground)

obstacle = chrono.ChBodyEasyBox(OBST_LEN_X, OBST_LEN_Y, OBST_HEIGHT,
                                1000.0, True, True, ground_mat)
obstacle.SetPos(OBST_CENTER)
obstacle.SetFixed(True)               # static ridge the rover climbs over
sys.Add(obstacle)

# === Rover === six-wheel rocker-bogie Curiosity with DC-motor straight-line driver
rover = robot.Curiosity(sys)
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
driver.SetSteering(0.0)                # zero steering -> drive straight forward
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_START, chrono.QUNIT))
rover.Update()                         # sync internal state after Initialize

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover crossing a long box obstacle")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-3.0, -7.0, 4.0), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.AddTypicalLights()                 # plain fill lighting (no shadow light: keeps render fast)
vis.AddGrid(1.0, 1.0, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Derived run constants === precomputed once before the loop
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once


# === Main loop === drive straight; render once per frame, batch physics between frames
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            rover.Update()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise
finally:
    vis.GetDevice().closeDevice()           # release the render window on any exit
