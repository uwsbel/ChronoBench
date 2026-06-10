"""
Curiosity rover on rigid terrain with a long box obstacle.
Turn 2: Initial position moved to (-5, 0.0, 0), zero steering (straight forward),
        added a long box obstacle for the rover to traverse.
"""
import os, math, csv
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
time_step = 1e-3
sim_end = 15.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Rover spawn (changed from (0, 0.2, 0) to (-5, 0.0, 0))
ROVER_X = -5.0
ROVER_Y = 0.0
ROVER_Z = 0.0   # z=0 places Curiosity on the ground surface (ground top at z=0)

# Obstacle: long box (placed ahead of rover at x ~ 0)
OBS_X = 0.0
OBS_Y = 0.0
OBS_Z = -0.5   # box top at z=0 (ground level), sits on terrain surface
OBS_LENGTH = 8.0   # long in x direction
OBS_WIDTH = 0.4
OBS_HEIGHT = 0.3

# === System setup (NSC — rover uses NSC) ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Long box obstacle ===
obstacle_mat = chrono.ChContactMaterialNSC()
obstacle_mat.SetFriction(0.7)
obstacle_mat.SetRestitution(0.0)
obstacle = chrono.ChBodyEasyBox(OBS_LENGTH, OBS_WIDTH, OBS_HEIGHT, 500, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVector3d(OBS_X, OBS_Y, OBS_Z + OBS_HEIGHT * 0.5))
obstacle.SetFixed(True)
system.Add(obstacle)

# === Curiosity rover ===
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(ROVER_X, ROVER_Y, ROVER_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - turn 2")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5, -8, 4), chrono.ChVector3d(-5, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===
frame = 0
chassis_body = rover.GetChassis().GetBody()  # cache: rover chassis for logging


while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    # Rover drives straight (zero steering input)
    driver.SetSteering(0.0)
    rover.Update()

    for _ in range(render_every):
        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break
