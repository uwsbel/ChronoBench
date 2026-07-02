import argparse
import math
import numpy as np
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


def main():
    # Create the PyChrono environment
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    # Create the HMMWV vehicle
    vehicle = veh.HMMWV_Full()
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChVector3d(0, 0.45, 0.8))
    vehicle.SetPowertrainType(veh.PowertrainType_TORQUE)
    vehicle.SetDriveType(veh.DrivelineType_AWD)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize(system)

    # Create the SCM deformable terrain
    terrain = veh.SCMTerrain(vehicle.GetSystem())
    terrain.SetSoilParameters(1800, 0.7, 0.001, 100000.0)
    terrain.SetMovingPatch(True)
    terrain.SetNumTiles(10, 10)
    terrain.SetTileLength(5.0, 5.0)
    terrain.SetTileSpacing(5.5, 5.5)
    terrain.SetPlotMesh(True)
    terrain.SetPlotSoil(True)
    terrain.SetPlotSinkage(True)
    terrain.Initialize(vehicle.GetSystem())

    # Create the interactive driver system
    driver = veh.Driver(vehicle)
    driver.SetInputType(veh.DriverInputType_KEYBOARD)
    driver.SetSteeringAngle(0.0)
    driver.SetThrottle(0.0)
    driver.SetBraking(0.0)

    # Create the Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("PyChrono HMMWV on SCM deformable terrain")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 1.5, 2.5), chrono.ChVector3d(0, 0.5, 0))
    vis.AddTypicalLights()

    # Simulation loop
    time = 0.0
    step = 0.001
    while vis.Run():
        time += step
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        driver.Synchronize(time)
        driver.Advance(step)
        vehicle.Synchronize(time, driver.GetInputs())
        terrain.Synchronize(time)
        system.DoStepDynamics(step)

    print("Simulation complete.")


if __name__ == "__main__":
    main()