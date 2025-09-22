import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh




chrono.SetChronoDataPath('../data/')  
chrono.Initialize()




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBodyEasyBox(10, 1, 0.25, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
ground.GetMaterialSurface().SetFriction(0.9)
ground.GetMaterialSurface().SetRestitution(0.2)
system.Add(ground)




rover = veh.Curiosity()
rover.SetChassisBodyFixed(False)
rover.SetInitPosition(chrono.ChVectorD(0, 0.2, 0))
rover.SetInitOrientation(chrono.Q_from_AngAxis(chrono.ChMatrix33Euler123(0, 0, 0)))
rover.SetSteering(chrono.ChFunction_Const(0))  
rover.SetThrottle(chrono.ChFunction_Const(0))  


for wheel in rover.GetWheels():
    system.Add(wheel)

system.Add(rover.GetChassisBody())
system.Add(rover.GetDriveline())




application = chronoirr.ChIrrApp(system, 'Curiosity Rover Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddLightWithShadow(chrono.ChVectorD(1.5, -2, 3), chrono.ChVectorD(0, 0, 0), 1, 1, 50, 50, 50, 40)
application.SetSymbolScale(0.05)
application.SetContactDrawMode(chronoirr.ChIrrApp.CONTACT_DRAW_SPHERE)
application.SetTimestep(0.01)
application.SetTryRealTime(True)




camera = application.GetSceneManager().addCameraSceneNode()
camera.setPosition(chrono.ChVectorD(5, 5, 5))
camera.lookAt(chrono.ChVectorD(0, 0, 0))




while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

application.Close()