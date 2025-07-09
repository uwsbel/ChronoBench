import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


rover = veh.Viper()
rover.SetContactMethod(chrono.ChContactMethod_SMC)
rover.SetChassisCollisionType(chrono.ChCollisionSystem.Type_BULLET)
rover.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
rover.Initialize()
sys.Add(rover)


driver = veh.ChDriver()
rover.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.01
time_end = 10.0
time = 0.0

while vis.Run():
    
    steering = chrono.ChFunction_Sine(0.1, 0.5)
    rover.SetSteering(steering.Get_y(time))

    
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    time += time_step

    
    if time > time_end:
        break