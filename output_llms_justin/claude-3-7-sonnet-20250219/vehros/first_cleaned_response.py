import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.ros as ros
import os
import math
import numpy as np





def main():
    
    system = chrono.ChSystemSMC()
    
    
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    system.SetSolverMaxIterations(150)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    
    
    
    
    
    
    terrain = veh.RigidTerrain(system)
    
    
    terrain_params = veh.RigidTerrainParams()
    terrain_params.SetDefaultMaterial(0.9,  
                                      0.7,  
                                      0.3)  
    terrain_params.SetDefaultColor(chrono.ChColor(0.8, 0.8, 1.0))
    terrain_params.SetDefaultTexture("textures/grass.jpg")
    
    
    terrain_length = 200.0
    terrain_width = 200.0
    patch_params = veh.RigidTerrainPatchParams(
        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
        chrono.ChVectorD(terrain_length, terrain_width, 2)
    )
    
    
    terrain.AddPatch(patch_params)
    terrain.Initialize()
    
    
    
    
    
    
    vehicle_file = veh.GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json")
    powertrain_file = veh.GetDataFile("hmmwv/powertrain/HMMWV_ShaftsPowertrain.json")
    tire_file = veh.GetDataFile("hmmwv/tire/HMMWV_TMeasyTire.json")
    
    
    vehicle = veh.HMMWV_Full(system)
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    
    
    vehicle.Initialize()
    
    
    for i in range(4):
        vehicle.GetTire(i).SetVisualizationType(veh.VisualizationType_MESH)
    
    
    
    
    
    
    driver_data = veh.PathFollowerDriverInputs(vehicle.GetVehicle())
    driver = veh.ChPathFollowerDriver(vehicle.GetVehicle())
    
    
    terrain_center = chrono.ChVectorD(0, 0, 0.5)
    target_speed = 10  
    
    
    path_points = []
    path_points.append(chrono.ChVectorD(0, 0, 0.5))
    path_points.append(chrono.ChVectorD(50, 0, 0.5))
    path_points.append(chrono.ChVectorD(100, 50, 0.5))
    path_points.append(chrono.ChVectorD(150, 0, 0.5))
    path_points.append(chrono.ChVectorD(200, 0, 0.5))
    
    
    path = chrono.ChBezierCurve(path_points)
    
    
    driver.SetDesiredSpeed(target_speed)
    driver.SetLookAheadDistance(10)
    driver.SetGains(0.8, 0.0, 0.0)  
    driver.Initialize()
    
    
    driver.SetPath(path)
    
    
    
    
    
    
    ros_manager = ros.ChROSNodeManager()
    
    
    ros_manager.RegisterHandler(
        ros.ChROSClockHandler(),
        "/clock"
    )
    
    
    ros_manager.RegisterHandler(
        ros.ChROSDriverInputsHandler(driver_data),
        "/vehicle/driver_inputs"
    )
    
    
    ros_manager.RegisterHandler(
        ros.ChROSVehicleStateHandler(vehicle.GetVehicle()),
        "/vehicle/state"
    )
    
    
    ros_manager.InitializeNode("pychrono_hmmwv_node")
    
    
    
    
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV Vehicle Simulation with ROS")
    vis.SetWindowSize(1280, 720)
    vis.Initialize()
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, -10, 2), chrono.ChVectorD(0, 0, 0))
    vis.AttachVehicle(vehicle.GetVehicle())
    
    
    
    
    
    
    end_time = 100  
    step_size = 1e-3  
    
    
    time = 0
    step_number = 0
    output_freq = 100  
    
    
    while time < end_time:
        
        ros_manager.Synchronize(time)
        
        
        throttle, steering, braking = driver.GetInputs()
        
        
        driver.Advance(step_size)
        vehicle.Synchronize(time, driver_data, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_data)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.Advance(step_size)
        
        
        if step_number % output_freq == 0:
            vehicle_pos = vehicle.GetVehicle().GetChassisBody().GetPos()
            vehicle_vel = vehicle.GetVehicle().GetChassisBody().GetPos_dt()
            vehicle_speed = vehicle_vel.Length()
            print(f"Time: {time:.3f} | Position: ({vehicle_pos.x:.2f}, {vehicle_pos.y:.2f}, {vehicle_pos.z:.2f}) | Speed: {vehicle_speed:.2f} m/s")
        
        
        time += step_size
        step_number += 1
        
        
        if not vis.Run():
            break
    
    
    ros_manager.FinalizeNode()
    
    print("Simulation completed!")
    return 0

if __name__ == "__main__":
    main()