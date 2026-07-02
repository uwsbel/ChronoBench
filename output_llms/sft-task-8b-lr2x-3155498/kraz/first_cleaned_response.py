import numpy as np
import pychrono as chrono
from pychrono.vehicle import ChSystem, ChDriver, ChVehicleModelData, ChWheeledVehicle, ChWheeledVehicleVisualSystemIrrlicht


def create_kraz_vehicle():
    vehicle = chrono.vehicle.Kraz()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(True)
    vehicle.SetInitPosition(chrono.ChVector3d(0, 0, 0.8))
    vehicle.Initialize()
    vehicle.SetPowertrainType(chrono.vehicle.PowertrainType_SIMPLE)
    vehicle.SetTireType(chrono.vehicle.TireType_RIGID)
    vehicle.SetTrackShoeVisualization(True)
    return vehicle


def create_terrain():
    terrain = chrono.ChBodyEasyBox(100, 0.1, 100, 1000, True, False)
    terrain.SetName("Rigid terrain")
    terrain.SetPos(chrono.ChVector3d(0, -0.55, 0))
    terrain.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    terrain.GetMaterialSurface().SetFriction(0.9)
    terrain.GetMaterialSurface().SetRestitution(0.01)
    return terrain


def create_visual_system(vehicle):
    vis = ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("PyChrono Kraz vehicle simulation")
    vis.SetWindowSize(1024, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0, 0.8, 2.5), 6.0, 0.5)
    vis.SetTimestep(0.01)
    vis.AddLightDirectional(chrono.ChVector3d(0.85, -1.0, 0.85), chrono.ChVector3d(0.2, 0.3, 0.8))
    vis.AddSkyBox()
    vis.AddGroundPlane(chrono.ChColor(0.8, 0.8, 0.9))
    vis.Initialize()
    vis.AddVehicle(vehicle)
    return vis


def main():
    system = chrono.ChSystemNSC()
    vehicle = create_kraz_vehicle()
    terrain = create_terrain()
    system.Add(vehicle.GetChassisBody())
    system.Add(terrain)
    driver = ChDriver(vehicle)
    vis = create_visual_system(vehicle)

    while vis.Run():
        time = system.GetChTime()
        driver.Synchronize(time)
        driver.Advance(0.01)
        vehicle.Synchronize(time, driver.GetInputs())
        vehicle.Advance(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(0.01)


if __name__ == "__main__":
    main()