import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os

def main():
    
    chrono.SetChronoDataPath(veh.GetDataFile(""))

    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) 
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)


    
    
    kraz_vehicle = veh.Kraz(system)

    
    
    
    
    initial_pos = chrono.ChVectorD(0, 0.6, 0) 
    initial_rot = chrono.Q_from_AngZ(0) 
    kraz_vehicle.SetInitPosition(chrono.ChCoordsysD(initial_pos, initial_rot))

    kraz_vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    kraz_vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    kraz_vehicle.SetDriveType(veh.DrivelineTypeWV_AWD) 
    kraz_vehicle.SetTireType(veh.TireModelType_TMEASY) 

    
    kraz_vehicle.Initialize()

    
    kraz_vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    kraz_vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz_vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    for axle in kraz_vehicle.GetAxles():
        for wheel in axle.GetWheels():
            wheel.SetTireVisualizationType(veh.VisualizationType_MESH)


    
    terrain = veh.RigidTerrain(system)
    
    
    
    contact_material = chrono.ChMaterialSurfaceNSC()
    contact_material.SetFriction(0.9)       
    contact_material.SetRestitution(0.01)   
    terrain.SetContactMaterial(contact_material)

    
    
    terrain_height = 0.0
    terrain_length = 200.0 
    terrain_width = 200.0  
    terrain.Initialize(terrain_height, terrain_length, terrain_width)
    
    
    texture_file = veh.GetDataFile("terrain/textures/tile4.jpg")
    if os.path.exists(texture_file):
        terrain.SetTexture(texture_file, 200, 200) 
    else:
        print(f"Warning: Terrain texture file not found: {texture_file}")


    
    
    
    

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Kraz Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(15, 10, 20), chrono.ChVectorD(0, 0, 0)) 
    vis.AddTypicalLights()
    
    
    vis.AttachSystem(system) 

    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.04)  
    driver.SetThrottleDelta(0.04)  
    driver.SetBrakingDelta(0.1)    
    driver.Initialize()


    
    
    timestep = 0.005  
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) 

    
    realtime_timer = chrono.ChRealtimeStepTimer()

    
    while vis.Run():
        time = system.GetChTime()

        
        driver_inputs = driver.GetInputs()
        steering_input = driver_inputs.m_steering
        throttle_input = driver_inputs.m_throttle
        braking_input = driver_inputs.m_braking

        
        driver.Synchronize(time)
        kraz_vehicle.Synchronize(time, steering_input, braking_input, throttle_input, terrain)
        terrain.Synchronize(time) 

        
        driver.Advance(timestep)
        kraz_vehicle.Advance(timestep)
        terrain.Advance(timestep)
        system.DoStepDynamics(timestep)

        
        vis.BeginScene()
        vis.Render()
        
        driver.DrawAll(vis) 
        vis.EndScene()

        
        realtime_timer.Spin(timestep)

    print("Simulation ended.")

if __name__ == '__main__':
    main()