import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Initialize Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create rigid terrain
    terrain = veh.RigidTerrain(system)
    terrain.SetTerrainType(veh.RigidTerrain.BOX)
    terrain.SetDimensions(20, 20, 0.2)
    terrain.Initialize()

    # Initialize Viper rover
    rover = veh.Viper(system)
    init_pos = chrono.ChVectorD(0, 0, 0.5)
    rover.Initialize(chrono.ChCoordsysD(init_pos, chrono.QUNIT))
    rover.SetTireType(veh.TireModelType_RIGID)

    # Create driver and set control parameters
    driver = veh.ViperDCMotorControl()
    rover.SetDriver(driver)
    driver.SetGains(0.4, 0.0)
    
    # Set up Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Viper Rover Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    # Simulation parameters
    time_step = 0.005
    current_time = 0
    ramp_duration = 5.0  # Steering ramp-up duration
    max_steering = 30.0   # Maximum steering angle (degrees)
    base_speed = 0.2     # Base forward speed (m/s)

    # Simulation loop
    while vis.Run():
        current_time = system.GetChTime()
        
        # Update steering angle gradually
        if current_time <= ramp_duration:
            steering = chrono.ChClamp(max_steering * (current_time / ramp_duration), 0.0, max_steering)
        else:
            steering = max_steering
            
        # Apply controls
        driver.SetSteering(steering)
        driver.SetForwardSpeed(base_speed)
        driver.Update(current_time)

        # Simulation advance
        vis.BeginScene()
        vis.Render()
        rover.Update()
        system.DoStepDynamics(time_step)
        vis.EndScene()

if __name__ == "__main__":
    main()