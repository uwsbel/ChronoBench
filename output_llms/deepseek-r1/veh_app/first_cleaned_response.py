import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    gator = veh.Gator(system)
    gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    gator.Initialize()
    
    
    gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.InteractiveDriverIRR(gator.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Gator Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1.5))
    vis.AddTypicalLights()

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorD(2, 2.5, 5), chrono.ChColor(0.8, 0.8, 1.0), 5.0)
    
    
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(1.5, 0, 1.0), chrono.Q_from_AngZ(0.2))
    camera = sens.ChCameraSensor(gator.GetChassisBody(), 30, offset_pose, 1280, 720, 1.0)
    camera.SetName("Vehicle Camera")
    camera.PushFilter(sens.ChFilterRGBA8())
    manager.AddSensor(camera)

    
    step_size = 0.002
    render_steps = 2

    while vis.Run():
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        
        terrain.Synchronize(time)
        
        
        gator.Synchronize(time, driver_inputs, terrain)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        manager.Update()
        
        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.RenderFrame(gator.GetChassisBody().GetFrame_REF_to_abs())
            vis.EndScene()
            
        step_number += 1

if __name__ == "__main__":
    main()