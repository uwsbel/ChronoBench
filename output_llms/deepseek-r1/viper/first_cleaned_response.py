import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), 
                            chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
    terrain.Initialize()
    
    
    rover = veh.Viper(system)
    rover.SetDriver(veh.ViperDCMotorControl())
    rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -0.2, 0.5), chrono.QUNIT)
    rover.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    rover.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper Rover Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0.5))
    vis.AddTypicalLights()
    
    
    step_size = 0.001
    end_time = 10.0
    current_time = 0.0
    
    
    while vis.Run() and current_time <= end_time:
        
        steering = 0.4 * chrono.ChSineStep(current_time, 1.0, 0.0, 3.0, -0.4)
        rover.GetDriver().SetSteering(steering)
        
        
        rover.Update(chrono.ChTimer(step_size))
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        system.DoStepDynamics(step_size)
        current_time += step_size

if __name__ == "__main__":
    main()