import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Set data path (adjust as needed for your installation)
chrono.SetChronoDataPath("path/to/chrono/data/")  # CHANGE THIS PATH

def main():
    # 1. Initialize simulation system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # 2. Create and initialize FEDA vehicle
    vehicle = veh.FEDA(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType.RIGID)
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # 3. Create rigid terrain with texture
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # Custom texture
    terrain.Initialize()

    # 4. Set up Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("FEDA Vehicle Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 3, 1.5), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, 5), chrono.ChVectorD(0, 0, 0.5), 50, 3, 30, 30, 512)

    # 5. Create interactive driver
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 6. Simulation loop
    step_size = 0.02  # 50 FPS
    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        vehicle.DriverInputs(driver_inputs)
        
        # Update vehicle systems
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        # Advance simulation
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)
        
        # Render at 50 FPS
        vis.Render()
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()