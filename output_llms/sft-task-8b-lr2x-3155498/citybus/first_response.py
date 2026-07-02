import argparse
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


def main():
    # Create the PyChrono system and visualization
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    # Create the terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.SetContactMaterial(patch_mat)
    patch = terrain.AddPatch(patch_mat, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(100, 100, 1.0), veh.GetDataFile("terrain/heightmap_100_100.png"))
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.55))
    terrain.Initialize()

    # Create the CityBus vehicle
    veh.CityBus.SetContactMethod(chrono.ChContactMethod_NSC)
    veh.CityBus.SetChassisFixed(False)
    veh.CityBus.SetInitPosition(chrono.ChVector3d(0, 0.5, veh.CityBus.GetChassisHeight()))
    veh.CityBus.SetInitFwdVel(0)
    veh.CityBus.SetEngineType(veh.EngineType.ENGINE_E4)
    veh.CityBus.SetDriveType(veh.DriveType.FWD)
    veh.CityBus.SetTireType(veh.TireModelType.TMEASY)
    veh.CityBus.Initialize()
    veh.CityBus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    veh.CityBus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    veh.CityBus.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
    veh.CityBus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    veh.CityBus.SetTireVisualizationType(chrono.VisualizationType_MESH)

    # Create the interactive driver system
    driver = veh.DriverCityBus()
    driver.SetSteeringAngleController(0.2, 1.0)
    driver.SetThrottleController(0.1, 0.0)
    driver.SetBrakingController(0.1, 0.0)
    driver.Initialize(veh.CityBus.GetVehicle())

    # Create the Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("PyChrono CityBus on Rigid Terrain")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10, 2, 1.5), chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()

    # Simulation loop
    time = 0.0
    step = 0.001
    while vis.Run():
        time += step
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        driver.Synchronize(time)
        veh.CityBus.Synchronize(time, driver.GetInputs())
        terrain.Synchronize(time)
        driver.Advance(step)
        veh.CityBus.Advance(step)
        terrain.Advance(step)


if __name__ == "__main__":
    main()