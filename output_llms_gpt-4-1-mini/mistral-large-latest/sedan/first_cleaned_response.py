import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("path/to/chrono/data")


system = chrono.ChSystemNSC()





ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.AddBody(ground)


sphere_radius = 0.5
sphere_mass = 10.0
sphere = chrono.ChBodyEasySphere(sphere_radius, sphere_mass, True, True)
sphere.SetPos(chrono.ChVectorD(0, sphere_radius, 0))
system.AddBody(sphere)




system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('PyChrono Simulation')
visualization.Initialize()
visualization.AddLightWithShadow(chrono.ChVectorD(2.0, 4.0, -1.0),    
                                 chrono.ChVectorD(2.0, -1.0, -1.0),   
                                 5,                                    
                                 9.5, 10.5,                           
                                 40)                                   
visualization.EnableShadows()


while visualization.Run():
    visualization.BeginScene()
    visualization.Render()
    system.DoStepDynamics(0.01)
    visualization.EndScene()