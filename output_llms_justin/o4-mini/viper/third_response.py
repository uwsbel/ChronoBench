import os
import math
import numpy as np

import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# ------------------------------------------------------------------------
# 1) Create the Chrono system
# ------------------------------------------------------------------------
system = chrono.ChSystemNSC()
# Use Bullet for contacts
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# Fix the method name: Set_G_acc instead of SetGravitationalAcceleration
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# ------------------------------------------------------------------------
# 2) Create SCM deformable terrain instead of a rigid ground box
# ------------------------------------------------------------------------
terrain = veh.SCMTerrain(system)

# Define the reference plane z=0  (normal = (0,0,1), offset = 0)
terrain.SetPlane(0, 0, 1, 0)

# Soil parameters: (Bekker Kphi, Bekker Kc, exponent n,
#                  cohesion, friction angle, Janosi shear, elastic K, damping R)
terrain.SetSoilParameters(
    2e5,                 # Bekker Kphi
    3e4,                 # Bekker Kc
    1.1,                 # exponent n
    0.0,                 # cohesion
    math.radians(30),    # friction angle
    2e5,                 # Janosi shear modulus
    3e4,                 # elastic stiffness
    0.01                 # viscous damping
)

# Initialize terrain grid: xmin, xmax, ymin, ymax, grid spacing
terrain.Initialize(-10, 10, -10, 10, 0.1)

# ------------------------------------------------------------------------
# 3) Instantiate the Viper rover and its DC‐motor driver
# ------------------------------------------------------------------------
rover  = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# Initialize rover pose at (0, 0.2, 0) with identity rotation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
# Fix the call to use ChCoordsysD
rover.Initialize(chrono.ChCoordsysD(init_pos, init_rot))

# ------------------------------------------------------------------------
# 4) Set up Irrlicht visualization
# ------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Viper rover – SCM deformable terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5),
              chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0,   0,   0.5),
                       3, 4, 10, 40, 512)

# ------------------------------------------------------------------------
# 5) Simulation loop
# ------------------------------------------------------------------------
time_step = 1e-3

while vis.Run():
    # 5.a) Constant zero steering
    driver.SetSteering(0.0)

    # 5.b) Update rover mechanics
    rover.Update()

    # 5.c) Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # 5.d) Advance dynamics
    system.DoStepDynamics(time_step)