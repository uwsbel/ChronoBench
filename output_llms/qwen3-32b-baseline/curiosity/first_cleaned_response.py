import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetCollisionModel().SetCollisionType(chrono.ChCollisionModelBase.ChCollisionType.STATIC)
system.Add(ground)


tex = chrono.ChTexture()
tex.SetTextureFilename(chrono.GetChronoDataPath() + 'concrete.jpg')
ground.AddAsset(tex)


rover = vehicle.Mars_rover("CuriosityRover")
rover.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
rover.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
rover.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
rover.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngY(0)))
system.Add(rover)


wheel = rover.GetWheel(0, vehicle.VehicleSide_LEFT)
wheel.GetSpindle().GetBody().GetCollisionModel().AddBox(0.3, 0.3, 0.3, chrono.ChVectorD(0, 0, 0))


driver = vehicle.ChDriver(rover)
driver.SetSteering(0.5)  
driver.SetThrottle(1.0)  


application = irr.ChIrrApp(system, 'Curiosity Rover Simulation', irr.dimension2du(1024, 768))
application.SetSkyBox()
application.SetCamera(chrono.ChVectorD(0, 3, -8), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
application.SetLightDirection(chrono.ChVectorD(1, -1, -1))
application.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono.png')
application.AddTypingText()
application.AddFPS()
application.SetShadows()


time_step = 0.001
time = 0

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    steering_angle = 0.5 * math.sin(time)
    driver.SetSteering(steering_angle)
    driver.Update(time_step)
    
    system.DoStepDynamics(time_step)
    time += time_step
    
    application.EndScene()