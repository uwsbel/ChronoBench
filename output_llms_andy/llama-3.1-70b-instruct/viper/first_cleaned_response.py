import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(mysystem, 10, 2, 10, 1000, True, True, chrono.ChVectorD(0, -3, 0))
ground.SetBodyFixed(True)


viper = veh.VehicleViper(mysystem)


viper.SetChassisPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))


driver = veh.VehicleDriverViper(viper)


driver.SetSteering(chrono.ChFunction_Const(chrono.CH_C_PI_4))
driver.SetThrottle(chrono.ChFunction_Const(0.5))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0))


vis.SetCameraUp(chrono.ChVectorD(0, 1, 0))
vis.SetCameraAimingMode(chronoirr.VisCameraAimingMode.FOLLOW_PATH)
vis.SetCameraPathSpeed(0.5)


vis.AddTypicalLights()


time_step = 0.01
time_end = 10


def update_steering(t):
    return chrono.CH_C_PI_4 * (1 - t / time_end)


for t in chrono.ChRealtimeStep(time_step):
    
    driver.SetSteering(chrono.ChFunction_Const(update_steering(t)))

    
    mysystem.Update()

    
    vis.Render()

    
    vis.PollEvents()