import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
import pychrono.vehicle as veh            # 1.  new import
from  pychrono import irrlicht as chronoirr

# ---------------------------------------------------------------------
# Create Chrono system
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# gravity
system.Set_G(chrono.ChVector3d(0, 0, -9.81))       # (fixed wrong call)

# collision envelope / margin (kept from original code)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# ---------------------------------------------------------------------
# Deformable SCM terrain (replaces rigid ground body)
# ---------------------------------------------------------------------
terrain = veh.SCMDeformableTerrain(system)

# reference plane z = 0
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0),
                                    chrono.ChQuaterniond(1, 0, 0, 0)))

length   = 20.0     # x–dimension (m)
width    = 20.0     # y–dimension (m)
delta    = 0.04     # mesh resolution (m)

terrain.Initialize(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0),
                                      chrono.ChQuaterniond(1, 0, 0, 0)),
                   length, width, delta)

# (some typical sand-like soil parameters)
terrain.SetSoilParametersBekker_Kphi_Kc_n(5.301e6, 102e3, 1.1)
terrain.SetSoilParametersMohrCoulomb(1.3, 30.0)
terrain.SetSoilParametersJanosiShear(0.01)
terrain.SetSoilParametersElastic_K(4.0e7)

# optional – speeds-up simulation when vehicle moves around
terrain.EnableMovingPatch(True)

# ---------------------------------------------------------------------
# Create Viper rover
# ---------------------------------------------------------------------
rover  = viper.Viper(system)
driver = viper.ViperDCMotorControl()
rover.SetDriver(driver)

# initial position & orientation
init_pos = chrono.ChVector3d(0.0, 0.20, 0.0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))   # (fixed wrong type)

# ---------------------------------------------------------------------
# Irrlicht run-time visualization
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover – SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0,   0,   0.5),
                       3, 4, 10, 40, 512)

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
time_step = 1e-3
driver.SetSteering(0.0)            # 3. constant steering (removed time-varying logic)

# ---------------------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------------------
while vis.Run():

    system.DoStepDynamics(time_step)   # advance the physical system
    rover.Update()                     # update rover internal states

    # Rendering
    vis.BeginScene()
    vis.Render()
    vis.EndScene()