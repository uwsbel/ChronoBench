import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    # Initialize Chrono system with gravity
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create ground plane
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    ground.GetCollisionModel().AddPlane(chrono.ChPlane(chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0)))
    ground.GetCollisionModel().BuildModel()
    my_system.Add(ground)

    # Initialize Viper rover
    viper = veh.Viper()
    viper.Initialize(my_system, ground, veh.VisualizationType_IRR)
    chassis = viper.GetChassis()
    chassis.SetPos(chrono.ChVectorD(0, 0.5, 0))  # Position above ground

    # Setup driver
    driver = veh.VehicleDriver()
    viper.InitializeDriver(driver)

    # Configure visualization
    visualiz = irr.ChIrrApp(my_system, 'Viper on Terrain', irr.dimension2du(800, 600))
    visualiz.AddTypicalLights()
    visualiz.AddTypicalCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, 0))
    visualiz.SetCameraSpeed(3)
    visualiz.AssetBind()
    visualiz.AssetUpdate()
    visualiz.SetShadowParams(100, 100)

    # Simulation parameters
    step_size = 0.01
    max_time = 60.0

    # Simulation loop
    while visualiz.Run() and my_system.GetChTime() < max_time:
        current_time = my_system.GetChTime()

        # Steering control (sinusoidal variation)
        period = 10.0
        max_steering = 0.5  # radians
        steer = max_steering * math.sin(2 * math.pi * current_time / period)
        driver.SetSteering(steer)
        driver.SetThrottle(0.5)  # Forward motion

        # Update driver and vehicle
        driver.Update(step_size)
        viper.Synchronize(current_time, driver)
        viper.Update(current_time)

        # Advance simulation and render
        my_system.DoStepDynamics(step_size)
        visualiz.BeginScene()
        visualiz.DrawAll()
        visualiz.EndScene()

if __name__ == '__main__':
    main()