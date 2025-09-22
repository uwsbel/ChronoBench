import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')  
system = chrono.ChSystemNSC()  


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))  
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(ground)


radius = 0.5
density = 1000  
sphere = chrono.ChBodyEasySphere(radius, density, True, True)
sphere.SetPos(chrono.ChVectorD(0, 5, 0))  
sphere.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(sphere)



gravity = chrono.ChVectorD(0, -9.81, 0)  
system.Set_G_acc(gravity)


time_step = 0.01  
end_time = 2.0  


application = chronoirr.ChIrrApp(system, "Falling Sphere Simulation", chronoirr.dimension2d_u(800, 600))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))
application.AddLight(chrono.ChVectorD(0, 10, -10), chrono.ChVectorD(1, 1, 1))


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    system.DoStepDynamics(time_step)

    
    if system.GetChTime() >= end_time:
        break


application.GetDevice().drop()