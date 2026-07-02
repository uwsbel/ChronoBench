import argparse
import math

import pychrono as chrono
import pychrono.vehicle as veh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--visualization", action="store_true", help="Enable visualization")
    parser.add_argument("-r", "--real_time", action="store_true", help="Run in real-time")
    args = parser.parse_args()

    
    rover = veh.Viper()
    rover.SetChassisColor(chrono.ChColor(0.5, 0.5, 0.5))
    rover.SetWheelColor(chrono.ChColor(0.2, 0.2, 0.2))
    rover.SetSuspensionColor(chrono.ChColor(0.8, 0.8, 0.8))
    rover.SetTrackColor(chrono.ChColor(0.1, 0.1, 0.1))

    
    terrain = veh.RigidTerrain(rover.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(100, 100, 0), patch_mat)
    terrain.Initialize()

    
    sys = rover.GetSystem()
    chrono.ChCollisionSystem.SetDefaultMotionStrategy(chrono.ChCollisionSystem.PointCloudMotion)
    chrono.ChCollisionSystem.SetDefaultSweepFlag(True)

    
    driver = veh.Driver(rover)
    driver.SetSteeringAngle(0.0)
    driver.SetThrottle(0.0)
    driver.SetBraking(0.0)

    
    if args.visualization:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("PyChrono Viper rover on rigid terrain")
        vis.SetWindowSize(1024, 768)
        vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.2), 6.0, 0.5)
        vis.Initialize()
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AddCamera()
        vis.AddTypicalLights()
        vis.AddVehicle(rover)
    else:
        vis = None

    
    time = 0.0
    step = 0.001
    while vis and vis.Run():
        time += step
        driver.Synchronize(time)
        rover.Synchronize(time, driver.GetInputs())
        terrain.Synchronize(time)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        driver.Advance(step)
        rover.Advance(step)
        terrain.Advance(step)

    print("Simulation complete")


if __name__ == "__main__":
    main()