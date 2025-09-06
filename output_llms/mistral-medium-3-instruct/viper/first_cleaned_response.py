import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np






step_size = 1e-3


t_end = 10


steering_time = 5






system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))






ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
ground.SetBodyFixed(True)
ground.GetVisualModel().SetTexture(veh.GetDataFile("textures/concrete.jpg"))
system.Add(ground)






viper = veh.Viper()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordinator(0, 0, 0.5))
viper.SetInitFwdVel(0.5)


viper.Initialize()


viper.GetSystem().AddToSystem(&system)






driver = veh.ChDriver()


driver.SetSteeringDelta(0.5)
driver.SetThrottleDelta(0.2)
driver.SetBrakingDelta(0.2)






app = chronoirr.ChIrrApp(system, "Viper Rover Simulation", chrono.ChVectorD(1280, 720))


app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(0, 5, 2), chrono.ChVectorD(0, 0, 0.5))
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()






time = 0

while app.GetDevice().run() and time < t_end:
    
    time = system.GetChTime()

    
    steering = np.sin(2 * np.pi * time / steering_time)

    
    driver.SetSteering(steering)
    driver.SetThrottle(0.5)

    
    driver.Synchronize(time)
    driver.Advance(step_size)

    
    viper.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    
    if int(time) % 1 == 0:
        print(f"Time: {time:.2f}s, Steering: {steering:.2f}, Position: {viper.GetChassisBody().GetPos().x:.2f}, {viper.GetChassisBody().GetPos().y:.2f}")