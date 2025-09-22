import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import numpy as np
import math

def main():
    print("Copyright (c) 2017 projectchrono.org")
    print("Chrono version: ", chrono.CHRONO_VERSION)
    
    
    chrono.SetChronoDataPath("data/")
    veh.SetDataPath("data/vehicle/")
    
    
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.ChQuaternionD(1, 0, 0, 0)))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
    
    
    terrain.SetSoilParameters(
        0.2e6,   
        0,       
        1.1,     
        0,       
        30,      
        0.01,    
        4e7,     
        3e4      
    )
    
    
    terrain.SetBulldozingParameters(
        55,      
        1,       
        50,      
        1.0,     
        0.5      
    )
    
    
    length = 20.0
    width = 20.0
    resolution = 0.02
    terrain.Initialize(length, width, resolution)
    
    
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000.2)
    
    
    driver = veh.ChInteractiveDriverIRR()
    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    
    driver.SetSteeringDelta(steering_time)
    driver.SetThrottleDelta(throttle_time)
    driver.SetBrakingDelta(braking_time)
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV SCM Deformable Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    
    
    vis.AttachVehicle(vehicle.GetVehicle())
    
    
    driver.Initialize()
    
    
    step_size = 1e-3
    tire_step_size = step_size
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    
    time = 0
    end_time = 30.0
    
    
    while vis.Run() and time < end_time:
        
        time = vehicle.GetSystem().GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        realtime_timer.Spin(step_size)
        
        
        if math.fmod(time, 1.0) < step_size:
            print(f"Time: {time:.2f}s, Vehicle Mass: {vehicle.GetVehicle().GetMass():.2f} kg")

if __name__ == "__main__":
    main()