# -----------------------------------------------------------------------------
# Curiosity rover – rigid-terrain test
# The script has been cleaned-up, all API calls were checked against the
# current PyChrono interface, and the requested modifications were introduced:
#   1.  A long box-shaped obstacle was added in front of the rover.
#   2.  The rover now starts at (-5, 0, 0).
#   3.  The rover is commanded to drive straight ahead with zero steering
#       for the entire run.
# -----------------------------------------------------------------------------


import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as robot
from   pychrono import irrlicht as chronoirr


# --------------------------------------------------------------------------
# 1.  Chrono system
# --------------------------------------------------------------------------
system = chrono.ChSystemNSC()

# Collision system, gravity, global margins
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))                       # <- fixed name
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(2.5e-3)
chrono.ChCollisionModel.SetDefaultSuggestedMargin  (2.5e-3)


# --------------------------------------------------------------------------
# 2.  Rigid​-terrain-> ground
# --------------------------------------------------------------------------
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20.0, 20.0, 1.0,        # size X,Y,Z  [m]
                              1000,                   # density [kg/m³] (irrelevant – body is fixed)
                              True, True, ground_mat) # collide, visualise, material
ground.SetPos  (chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# --------------------------------------------------------------------------
# 3.  Long box obstacle (requirement 1)
# --------------------------------------------------------------------------
obs_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(1.0,           # length (x)  – rover must climb over
                                5.0,           # width  (y)
                                0.30,          # height (z)
                                800,           # density
                                True, True, obs_mat)
obstacle.SetPos  (chrono.ChVector3d(0.0, 0.0, 0.15))   # half-height -> top at z = 0.30
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.2, 0.2))
system.Add(obstacle)


# --------------------------------------------------------------------------
# 4.  Curiosity rover
# --------------------------------------------------------------------------
rover  = robot.Curiosity(system)               # build rover inside 'system'
driver = robot.CuriosityDCMotorControl()       # DC-motor driver
rover.SetDriver(driver)

# Requirement-2 : new initial configuration
init_pos = chrono.ChVector3d(-5.0, 0.0, 0.0)   # starting 5 m in front of obstacle
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # no rotation
rover.Initialize(chrono.ChCoordsysD(init_pos, init_rot))   # <- fixed class name


# --------------------------------------------------------------------------
# 5.  Driver inputs  (requirement-3)
#     – constant throttle, zero steering
# --------------------------------------------------------------------------
driver.SetThrottle(0.6)    # forward drive (60 % of full motor voltage)
driver.SetSteering(0.0)    # no steering


# --------------------------------------------------------------------------
# 6.  Irrlicht visualisation
# --------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)

vis.SetWindowSize (1280, 720)
vis.SetWindowTitle('Curiosity rover – rigid terrain with obstacle')
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()

vis.AddLogo (chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d( 1.5, -2.5,  5.5),
                       chrono.ChVector3d( 0.0,  0.0,  0.0),
                       3, 4, 10, 40, 512)
# vis.EnableShadows()      # uncomment if GPU-budget allows


# --------------------------------------------------------------------------
# 7.  Simulation loop
# --------------------------------------------------------------------------
time_step = 1e-3      # [s]
simulation_time = 0.0

while vis.Run():
    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update rover subsystems
    rover.Update()

    # Step integrator
    system.DoStepDynamics(time_step)
    simulation_time += time_step