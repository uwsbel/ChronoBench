import pychrono as chrono
from pychrono import vehicle, irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


ground = vehicle.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -1))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(ground)


viper = vehicle.ViperRover()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisCollisionType(vehicle.CollisionType_NONE)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
viper.SetMotorType(vehicle.MotorType_SIMPLE)
viper.Initialize()
viper.AddVisualizationAssets(vehicle.VisualizationType_MESH)


driver = vehicle.ChDriver(viper)
driver.Initialize()


application = chronoirr.ChIrrApp(system, 'Viper Rover Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalCamera(chronoirr.vector3df(0, 1.5, 2.0), chronoirr.vector3df(0, 0, 1))
application.SetShowInfos(True)
application.SetContactsDrawMode(chronoirr.ECONTACTDRAWMODE_POINTS)
application.SetCollisionsDrawMode(chronoirr.ECONTACTDRAWMODE_POINTS)
application.Initialize()


time_step = 0.001
max_steering = 0.5
steering_duration = 5.0


while application.GetDevice().run():
    current_time = system.GetChTime()
    
    
    steering = min(current_time / steering_duration * max_steering, max_steering)
    throttle = 0.5
    braking = 0.0
    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)
    
    
    viper.Synchronize(current_time)
    system.DoStepDynamics(time_step)
    viper.Advance(time_step)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()