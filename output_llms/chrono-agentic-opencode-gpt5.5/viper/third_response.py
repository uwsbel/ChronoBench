"""Viper rover on SCM deformable terrain.

This standalone PyChrono NSC simulation builds the catalog Viper rover, replaces
the rigid ground with Bekker-Wong SCM soft soil, and drives the rover straight
with constant zero steering so the wheels deform the terrain and leave tracks.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.vehicle as veh


# === Constants === physical and rendering parameters kept visible and fixed
TIME_STEP = 1.0e-3
SIM_END = 6.0
TERRAIN_LENGTH = 14.0
TERRAIN_WIDTH = 4.0
TERRAIN_RESOLUTION = 0.04
TERRAIN_PLANE_Z = -0.5
STEERING = 0.0


# === System === NSC rover system with Bullet collision for wheel-soil contact
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Rover === catalog Viper with its built-in bodies, suspension, and motors
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))
chassis_body = rover.GetChassis().GetBody()  # cache: chassis pose logged every step


# === SCM terrain === deformable soil patch replacing the fixed ground body
terrain = veh.SCMTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_PLANE_Z)))
terrain.SetSoilParameters(0.2e6, 0.0, 1.1, 0.0, 30.0, 0.01, 4.0e7, 3.0e4)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0.0, 20000.0)
terrain.SetMeshWireframe(True)
terrain.AddMovingPatch(chassis_body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_RESOLUTION)


# === Visualization === Irrlicht window initialized before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - SCM deformable terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3,
    4,
    10,
    40,
    512,
)


# === Main loop === straight steering command, rover update, render, and step


while vis.Run() and system.GetChTime() < SIM_END:
    driver.SetSteering(STEERING)
    rover.Update()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(TIME_STEP)
