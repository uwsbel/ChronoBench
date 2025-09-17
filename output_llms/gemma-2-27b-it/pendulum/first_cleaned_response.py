import pychrono as chrono
import pychrono.irrlicht as chronoirr







mysystem = chrono.ChSystemNSC()


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, material)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
mysystem.Add(ground)


bob = chrono.ChBodyEasySphere(0.5, 10, True, True, material)
bob.SetPos(chrono.ChVectorD(0, 1, 0))
mysystem.Add(bob)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, bob, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
mysystem.Add(joint)







myapplication = chronoirr.ChIrrApp(mysystem, 'Simple Pendulum', chronoirr.dimension2du(800, 600))


myapplication.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
myapplication.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))






while (myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()

    
    print(f"Position: {bob.GetPos()}, Velocity: {bob.GetPos_dt()}")