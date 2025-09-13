import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # 1. Initialize PyChrono environment
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # 2. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 50)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 50)
    terrain.Initialize()

    # 3. Create and initialize UAZBUS vehicle
    vehicle = veh.UAZBUS()
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5)))
    vehicle.Initialize(veh.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
    vehicle.SetChassisFixed(False)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(0.002)
    vehicle.InitializeTires()

    # 4. Create driver system
    driver = veh.ChIrrGuiDriver(irr_env)
    steering_time = 1.0  # seconds
    driver.SetSteeringDelta(0.02 / steering_time)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # 5. Set up Irrlicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('UAZBUS Simulation')
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AddLogo()
    vis.AttachVehicle(vehicle.GetVehicle())

    # 6. Simulation loop
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        time = system.GetChTime()
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        # Advance simulation
        driver.Synchronize(time)
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()