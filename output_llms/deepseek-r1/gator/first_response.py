import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

def main():
    # Initialize Chrono system
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create and initialize Gator vehicle
    gator = veh.Gator(system)
    gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngZ(0)))
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(0.001)
    gator.Initialize()
    
    # Set visualization modes
    gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create interactive driver
    driver = veh.ChIrrGuiDriver()
    steering_axis = chrono.ChVectorD(0, 0, 0.2)
    throttle_axis = chrono.ChVectorD(0, 0, 0.3)
    brake_axis = chrono.ChVectorD(0, 0, 0.4)
    driver.SetInputMode(veh.InputMode_KEYBOARD)
    driver.SetInputReference(steering_axis, throttle_axis, brake_axis)

    # Set up Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Gator Vehicle Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, -10, 1.5))
    vis.AddLightWithShadow(chrono.ChVectorD(10, 6, 8), chrono.ChVectorD(0, 0, 0), 50)
    vis.AddTypicalLights()
    vis.EnableShadows()

    # Attach driver to visualization system
    vis.AddUserEventReceiver(driver)

    # Simulation parameters
    step_size = 0.002
    render_step = 1.0 / 50  # 50 FPS
    time = 0
    frame = 0

    # Main simulation loop
    while vis.Run():
        time = system.GetChTime()
        
        # Render scene
        if frame % int(render_step / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        gator.Synchronize(time, driver_inputs)
        
        # Advance simulation
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.Synchronize(time, driver_inputs)
        
        frame += 1

if __name__ == "__main__":
    main()