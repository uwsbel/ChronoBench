import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    ground = chrono.ChBodyEasyPlane(20, 20, 100, False, True)
    ground.SetPos(chrono.ChVectorD(0, -1, 0))
    sys.Add(ground)

    
    vehicle = veh.Viper(sys)
    vehicle.Initialize(
        chrono.ChVectorD(0, 0.1, 0.5),
        chrono.Q_from_AngY(0),
        False,
        veh.VehicleDifferentialType.DEFAULT,
        'concrete'
    )
    driver = vehicle.GetDriver()

    
    vis = irr.ChIrrApp(sys, 'Viper on Rigid Terrain', irr.dimension2du(800, 600))
    vis.AddTypicalLights()
    vis.AddTypicalCamera(chrono.ChVectorD(5, 5, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddAll()
    vis.SetTimestep(0.01)  

    
    total_time = 60.0
    steering_period = 5.0  

    
    while vis.Run():
        current_time = sys.GetChTime()
        if current_time >= total_time:
            break

        
        steering = 0.5 * math.sin(2 * math.pi * current_time / steering_period)
        driver.SetSteering(steering)

        
        vehicle.Update(current_time)
        sys.DoStepDynamics(vis.GetTimestep())

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == '__main__':
    main()