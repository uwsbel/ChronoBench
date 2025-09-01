import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)



ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
my_system.Add(ground)


sphere = chrono.ChBodyEasySphere(1, 100, True, True, material)
sphere.SetPos(chrono.ChVectorD(0, 2, 0))
my_system.Add(sphere)



force = chrono.ChForce(chrono.ChForceType_FORCE)
force.SetDir(chrono.ChVectorD(0, -1, 0))
force.SetMforce(10)
sphere.AddForce(force)


my_system.SetContactMethod(chrono.ChContactMethod.ChContactMethodNSC)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('PyChrono Simulation')
vis.Initialize()
vis.Run()


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.Render()