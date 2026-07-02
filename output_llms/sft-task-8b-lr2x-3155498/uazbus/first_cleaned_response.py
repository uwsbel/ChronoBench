import pychrono as chrono
import numpy as np


def create_rigid_terrain():
    terrain = chrono.ChSystemNSC()
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    terrain.SetSolverType(chrono.ChSolver.Type_ITERATIVE)
    terrain.SetMaxIterations(100)
    terrain.SetTolerance(1e-8)

    ground = chrono.ChBodyEasyBox(100, 100, 0.1, 1000, True, True)
    ground.SetName("Rigid terrain")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, -0.05, 0))
    ground.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))
    terrain.Add(ground)
    return terrain, ground


def create_uazbus_vehicle(terrain):
    vehicle = chrono.ChVehicleSystem()
    vehicle.SetUseVisualization(True)

    chassis = chrono.ChBodyEasyBox(2.5, 1.2, 0.8, 1000, True, True)
    chassis.SetName("UAZBUS chassis")
    chassis.SetPos(chrono.ChVector3d(0, 0.4, 0))
    chassis.SetRot(chrono.QuatFromAngleX(-0.1))
    chassis.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.3, 0.1))
    chassis.GetVisualShape(0).SetOpacity(0.8)
    vehicle.AddBody(chassis)

    driver = chrono.ChDriverSystem(vehicle)
    driver.SetVehicle(vehicle)
    driver.SetMaxSteeringAngle(np.pi / 6)
    driver.SetMaxThrottle(1.0)
    driver.SetMaxBrakingTorque(10000)
    driver.SetSteeringInput(0.0)
    driver.SetThrottleInput(0.0)
    driver.SetBrakingInput(0.0)
    return vehicle, chassis, driver


def create_irrlicht_visual(system, terrain, vehicle, chassis):
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("PyChrono UAZBUS Simulation")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10, 5, 5), chrono.ChVector3d(0, 0, 0.5))
    vis.AddLightDirectional(chrono.ChVector3d(0.5, 0.5, 0.5), chrono.ChVector3d(0.2, 0.2, 0.2))
    vis.AddLightPoint(chrono.ChVector3d(0, 0.5, 0.5), 10, chrono.ChColor(0.8, 0.8, 0.8))
    return vis


def main():
    terrain, ground = create_rigid_terrain()
    vehicle, chassis, driver = create_uazbus_vehicle(terrain)
    vis = create_irrlicht_visual(terrain, terrain, vehicle, chassis)

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        terrain.DoStepDynamics(0.01)
        driver.Update()
        vis.Synchronize(terrain.GetChTime())
        vis.Advance(0.01)


if __name__ == "__main__":
    main()