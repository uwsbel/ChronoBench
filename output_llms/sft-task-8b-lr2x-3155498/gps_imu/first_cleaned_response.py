import argparse
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


def main():
    
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    vehicle = veh.HMMWV_Full(sys)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.Initialize(veh.GetDataFile("terrain/heightmap_20x20.mtl"), patch_mat)

    
    driver = veh.Driver(vehicle, terrain)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("PyChrono HMMWV Full Vehicle Simulation")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0.7, 1.2), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    
    imu = veh.IMU_Sensors(vehicle.GetChassisBody(), "IMU")
    imu.SetMountPoint(chrono.ChVector3d(0, 0, 0))
    imu.SetFrame(chrono.ChFrame3d(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    imu.SetUpdateRate(100)
    imu.SetOutputFile("imu_data.txt")

    gps = veh.GPS_Sensors(vehicle.GetChassisBody(), "GPS")
    gps.SetMountPoint(chrono.ChVector3d(0, 0, 0))
    gps.SetFrame(chrono.ChFrame3d(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    gps.SetUpdateRate(100)
    gps.SetOutputFile("gps_data.txt")

    
    step_size = 1.0 / 100
    max_time = 20.0
    output_interval = 10

    
    print("---------- PyChrono HMMWV Full Vehicle Simulation ----------")
    print("Use arrow keys to control the vehicle. Press 'Q' to quit.")
    print("------------------------------------------------------------")
    print("Vehicle mass: %.2f kg" % vehicle.GetVehicleMass())
    print("------------------------------------------------------------")

    while vis.Run():
        time = vis.GetSimulationTime()
        if time >= max_time:
            break

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs())
        imu.Synchronize(time)
        gps.Synchronize(time)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        imu.Advance(step_size)
        gps.Advance(step_size)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        if time % output_interval < step_size:
            print("Time: %.2f s, Vehicle mass: %.2f kg" % (time, vehicle.GetVehicleMass()))

    print("Simulation complete.")
    print("IMU and GPS data saved to imu_data.txt and gps_data.txt.")
    print("------------------------------------------------------------")


if __name__ == "__main__":
    main()