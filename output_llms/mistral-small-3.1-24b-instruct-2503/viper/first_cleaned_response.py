import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import pychrono.vehicle as veh


chrono.SetChronoDataPath('path_to_your_chrono_data')  
system = chrono.ChSystemNSC()


ground_body = chrono.ChBodyEasyBox(10, 10, 0.5, 1000, True, True)
ground_body.SetPos(chrono.ChVectorD(0, 0, -0.25))
ground_body.SetBodyFixed(True)
system.Add(ground_body)


system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


viper_robot = veh.ChViper()
viper_robot.SetChassisBody(system)
viper_robot.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
viper_robot.SetInitOrientation(chrono.Q_from_AngAxis(chrono.ChVectorD(1, 0, 0), chrono.CHRONO_PI / 2))
viper_robot.SetDriver(veh.ChDriverVehicleRigid)
viper_robot.Initialize(system)


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetCamera(chrono.ChVectorD(5, -5, 5), chrono.ChVectorD(0, 0, 0))
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Viper Rover Simulation')
visualization.SetLightDirection(chrono.ChVectorD(1, -1, -1))
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()


steering_angle = 0
steering_change_rate = 0.01  
simulation_time = 10  
timestep = 0.01  


while visualization.Run():
    
    steering_angle += steering_change_rate
    if steering_angle > 0.5:
        steering_change_rate = -0.01
    if steering_angle < -0.5:
        steering_change_rate = 0.01

    
    viper_robot.SetSteering(steering_angle)

    
    system.DoStepDynamics(timestep)

    
    visualization.Render()

    
    if system.GetChTime() >= simulation_time:
        break


visualization.Close()