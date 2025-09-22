import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    ground.GetCollisionModel().AddPlane(chrono.ChPlane(chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0)))
    ground.GetCollisionModel().BuildModel()
    my_system.Add(ground)

    
    viper = veh.Viper()
    viper.Initialize(my_system, ground, veh.VisualizationType_IRR)
    chassis = viper.GetChassis()
    chassis.SetPos(chrono.ChVectorD(0, 0.5, 0))  

    
    driver = veh.VehicleDriver()
    viper.InitializeDriver(driver)

    
    visualiz = irr.ChIrrApp(my_system, 'Viper on Terrain', irr.dimension2du(800, 600))
    visualiz.AddTypicalLights()
    visualiz.AddTypicalCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, 0))
    visualiz.SetCameraSpeed(3)
    visualiz.AssetBind()
    visualiz.AssetUpdate()
    visualiz.SetShadowParams(100, 100)

    
    step_size = 0.01
    max_time = 60.0

    
    while visualiz.Run() and my_system.GetChTime() < max_time:
        current_time = my_system.GetChTime()

        
        period = 10.0
        max_steering = 0.5  
        steer = max_steering * math.sin(2 * math.pi * current_time / period)
        driver.SetSteering(steer)
        driver.SetThrottle(0.5)  

        
        driver.Update(step_size)
        viper.Synchronize(current_time, driver)
        viper.Update(current_time)

        
        my_system.DoStepDynamics(step_size)
        visualiz.BeginScene()
        visualiz.DrawAll()
        visualiz.EndScene()

if __name__ == '__main__':
    main()