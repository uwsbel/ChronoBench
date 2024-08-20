import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC()  


application = chronoirr.ChIrrApp(chrono.ChSystemNSC(), "PyChrono Simulation", chronoirr.dimension2du(800, 600))
application.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono.png")
application.SetSkyBox(chrono.GetChronoDataPath() + "skybox.jpg")
application.SetCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 1, 0))
application.SetLight(chrono.ChVectorD(0, 10, -10), chrono.ChVectorD(1, 1, 1))


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)  
chrono.ChSystemNSC().Add(ground)


sphere_radius = 0.5
sphere = chrono.ChBodyEasySphere(sphere_radius, 1000, True, True)
sphere.SetPos(chrono.ChVectorD(0, 5, 0))  
chrono.ChSystemNSC().Add(sphere)


sphere.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
sphere.GetMaterialSurface().SetFriction(0.4)
sphere.GetMaterialSurface().SetRestitution(0.6)


chrono.ChSystemNSC().SetStep(0.01)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    chrono.ChSystemNSC().DoStepDynamics(0.01)  


application.GetDevice().drop()