import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




def main():
    
    chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), 
                            chrono.ChVectorD(0, 0, 1), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    truck = veh.MAN_10t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))
    truck.SetTireType(veh.TireModelType_TMEASY)
    truck.Initialize()

    
    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.InteractiveDriverIRR(truck.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('MAN 10t Truck Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, -5), chrono.ChVectorD(0, 0, 0), 
                          50, 5, 50, 35, 512, chrono.ChColor(0.8, 0.8, 1.0))

    
    step_size = 0.005
    render_step = 1.0 / 50  

    truck.GetVehicle().EnableRealtime(True)
    while vis.Run():
        time = system.GetChTime()
        
        
        driver.Synchronize(time)
        
        
        truck.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChVectorD(0,0,0), chrono.QUNIT, 2)  
        vis.EndScene()
        
        
        driver.DrawHUD(True, True, True, 120, 40, 12, 0.5, 0.5)




if __name__ == "__main__":
    main()