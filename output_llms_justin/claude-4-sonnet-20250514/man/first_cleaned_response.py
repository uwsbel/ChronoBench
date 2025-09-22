import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import numpy as np

def main():
    
    chrono.SetChronoDataPath("data/")
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
    
    
    terrain = veh.RigidTerrain(system)
    
    
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                            chrono.QUNIT),
                           200, 200)  
    
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    vehicle = veh.MAN_10t()
    
    
    init_pos = chrono.ChVectorD(0, 0.5, 0)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    vehicle.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetWheelCollisionType(veh.CollisionType_NONE)
    
    
    vehicle.Initialize(system)
    
    
    tire_FL = veh.TMeasyTire("FL")
    tire_FR = veh.TMeasyTire("FR")
    tire_RL = veh.TMeasyTire("RL")
    tire_RR = veh.TMeasyTire("RR")
    
    
    tire_FL.SetTMeasyParams(veh.GetDataFile("vehicle/hmmwv/tire/HMMWV_TMeasy.json"))
    tire_FR.SetTMeasyParams(veh.GetDataFile("vehicle/hmmwv/tire/HMMWV_TMeasy.json"))
    tire_RL.SetTMeasyParams(veh.GetDataFile("vehicle/hmmwv/tire/HMMWV_TMeasy.json"))
    tire_RR.SetTMeasyParams(veh.GetDataFile("vehicle/hmmwv/tire/HMMWV_TMeasy.json"))
    
    tire_FL.Initialize(vehicle.GetWheelBody(veh.FRONT_LEFT), veh.LEFT)
    tire_FR.Initialize(vehicle.GetWheelBody(veh.FRONT_RIGHT), veh.RIGHT)
    tire_RL.Initialize(vehicle.GetWheelBody(veh.REAR_LEFT), veh.LEFT)
    tire_RR.Initialize(vehicle.GetWheelBody(veh.REAR_RIGHT), veh.RIGHT)
    
    tire_FL.SetVisualizationType(veh.VisualizationType_MESH)
    tire_FR.SetVisualizationType(veh.VisualizationType_MESH)
    tire_RL.SetVisualizationType(veh.VisualizationType_MESH)
    tire_RR.SetVisualizationType(veh.VisualizationType_MESH)
    
    
    driver = veh.ChInteractiveDriverIRR()
    
    
    driver.SetSteeringDelta(0.02)  
    driver.SetThrottleDelta(0.02)  
    driver.SetBrakingDelta(0.02)   
    
    
    driver.Initialize()
    
    
    vis = veh.ChWheeledVehicleIrrApp(vehicle, 'MAN 10t Truck Simulation')
    
    
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('MAN 10t Truck - PyChrono Simulation')
    
    
    vis.SetChaseCamera(chrono.ChVectorD(-8.0, 3.0, 0.0),  
                       6.0,                                 
                       0.5)                                 
    
    
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    
    
    vis.AddTypicalLights()
    vis.AddLight(chrono.ChVectorD(30, 100, 30), 290, chrono.ChColor(0.7, 0.7, 0.7))
    vis.AddLight(chrono.ChVectorD(-30, 100, -30), 190, chrono.ChColor(0.7, 0.7, 0.7))
    
    
    vis.AddSkyBox(veh.GetDataFile("skybox/"))
    
    
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    
    
    time_step = 1e-3
    time_end = 60.0  
    
    
    vis.Initialize()
    vis.AddTypicalCamera(irr.vector3df(0, 5, -10))
    
    
    print(f"Vehicle mass: {vehicle.GetVehicle().GetMass():.2f} kg")
    print("Controls:")
    print("  W/S - Throttle/Brake")
    print("  A/D - Steering left/right")
    print("  Space - Emergency brake")
    
    
    time = 0
    while vis.GetDevice().run() and time < time_end:
        
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        tire_FL.Synchronize(time, terrain)
        tire_FR.Synchronize(time, terrain)
        tire_RL.Synchronize(time, terrain)
        tire_RR.Synchronize(time, terrain)
        vis.Synchronize(time, driver_inputs)
        
        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        tire_FL.Advance(time_step)
        tire_FR.Advance(time_step)
        tire_RL.Advance(time_step)
        tire_RR.Advance(time_step)
        system.DoStepDynamics(time_step)
        vis.Advance(time_step)
        
        
        vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        vis.DrawAll()
        
        
        if int(time * 10) % 10 == 0:  
            speed = vehicle.GetVehicle().GetPointVelocity(chrono.ChVectorD(0, 0, 0)).Length() * 3.6  
            pos = vehicle.GetVehicle().GetPos()
            
            info_text = f"Time: {time:.1f}s | Speed: {speed:.1f} km/h | Position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})"
            print(info_text)
        
        vis.EndScene()
    
    print("Simulation completed!")

if __name__ == "__main__":
    main()