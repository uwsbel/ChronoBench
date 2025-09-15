import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    terrain = veh.RigidTerrain(system)
    terrain.SetTerrainType(veh.RigidTerrain.BOX)
    terrain.SetDimensions(20, 20, 0.2)
    terrain.Initialize()

    
    rover = veh.Viper(system)
    init_pos = chrono.ChVectorD(0, 0, 0.5)
    rover.Initialize(chrono.ChCoordsysD(init_pos, chrono.QUNIT))
    rover.SetTireType(veh.TireModelType_RIGID)

    
    driver = veh.ViperDCMotorControl()
    rover.SetDriver(driver)
    driver.SetGains(0.4, 0.0)
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Viper Rover Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    time_step = 0.005
    current_time = 0
    ramp_duration = 5.0  
    max_steering = 30.0   
    base_speed = 0.2     

    
    while vis.Run():
        current_time = system.GetChTime()
        
        
        if current_time <= ramp_duration:
            steering = chrono.ChClamp(max_steering * (current_time / ramp_duration), 0.0, max_steering)
        else:
            steering = max_steering
            
        
        driver.SetSteering(steering)
        driver.SetForwardSpeed(base_speed)
        driver.Update(current_time)

        
        vis.BeginScene()
        vis.Render()
        rover.Update()
        system.DoStepDynamics(time_step)
        vis.EndScene()

if __name__ == "__main__":
    main()