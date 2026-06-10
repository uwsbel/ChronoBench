"""Viper rover driving straight on SCM deformable terrain.

This PyChrono 9.0.0 simulation uses an NSC system with Bullet collision, a
built-in robot.Viper rover, a Viper DC motor driver, and vehicle.SCMTerrain soft
soil. The rover keeps steering at zero so the powered wheels roll forward and
leave straight tracks in the deformable terrain.
"""

import traceback

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.vehicle as veh


# === Constants ===
# Named values keep the rover, terrain, and visualization explicit.
TIME_STEP = 1.0e-3
SIM_END = 6.0
TERRAIN_LENGTH = 14.0
TERRAIN_WIDTH = 4.0
TERRAIN_DELTA = 0.02
TERRAIN_PLANE_Z = -0.5
STEERING = 0.0
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720


def main():
    # === System & collision ===
    # Viper rovers use an NSC system with Bullet contact for wheel-soil contact.
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # === SCM deformable terrain ===
    # The soft-soil grid is centered under the rover and visualizes pressure.
    terrain = veh.SCMTerrain(system)
    terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_PLANE_Z)))
    terrain.SetSoilParameters(0.2e6, 0.0, 1.1, 0.0, 30.0, 0.01, 4.0e7, 3.0e4)
    terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0.0, 20000.0)
    terrain.SetMeshWireframe(True)
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 3.0)
    terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_DELTA)

    # === Rover and driver ===
    # The built-in Viper constructs its chassis, suspension, wheels, and motors.
    rover = robot.Viper(system)
    driver = robot.ViperDCMotorControl()
    rover.SetDriver(driver)
    init_pos = chrono.ChVector3d(0.0, 0.2, 0.0)
    init_rot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
    rover.Initialize(chrono.ChFramed(init_pos, init_rot))

    # === Visualization ===
    # Irrlicht is initialized before scene elements are added.
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT)
    vis.SetWindowTitle("Viper rover - SCM terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-3.0, 3.0, 1.8), chrono.ChVector3d(1.5, 0.0, 0.2))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        chrono.ChVector3d(1.5, -2.5, 5.5),
        chrono.ChVector3d(0.0, 0.0, 0.5),
        3,
        4,
        10,
        40,
        512,
    )
    vis.AddGrid(
        1.0,
        1.0,
        14,
        4,
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.01), chrono.QUNIT),
        chrono.ChColor(0.35, 0.35, 0.35),
    )


    # === Main loop ===
    # Keep steering constant at zero and update the rover driver every step.
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            driver.SetSteering(STEERING)
            rover.Update()

            vis.BeginScene()
            vis.Render()
            vis.EndScene()


            system.DoStepDynamics(TIME_STEP)
    except (RuntimeError, ValueError, OSError) as exc:  # solver, bad state, or output failure
        traceback.print_exc()
        raise exc
    finally:
        pass


if __name__ == "__main__":
    main()
