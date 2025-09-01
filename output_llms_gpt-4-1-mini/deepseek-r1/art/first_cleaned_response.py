import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


def main():
    
    chrono.SetChronoDataPath("path/to/chrono/data/")
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    initial_pos = chrono.ChVectorD(0, 1, 0)  
    initial_rot = chrono.Q_ROTATE_Y_TO_Z  
    contact_method = chrono.ChContactMethod_NSC
    vehicle = veh.ARTVehicle(system, contact_method)
    vehicle.Initialize(chrono.ChCoordsysD(initial_pos, initial_rot))
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                            chrono.ChVectorD(0, 0, 0), 
                            chrono.ChVectorD(0, 1, 0), 
                            100, 100)  
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    terrain.Initialize()
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ARTcar Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(-8, 2, 0), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    
    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetThrottleDelta(1.0/50)
    driver.SetSteeringDelta(1.0/50)
    driver.SetBrakingDelta(1.0/50)
    driver.Initialize()
    
    
    step_size = 0.02  
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        speed = vehicle.GetChassisBody().GetPos_dt().Length() * 3.6  
        vis.GetGUIEnvironment().addStaticText(
            f"Time: {time:.2f}s | Speed: {speed:.2f} km/h", 
            irr.recti(10, 10, 400, 40))
        
        
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()