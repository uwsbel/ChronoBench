import chrono as chrono
import irrlicht
import math


chrono.Initialize()


sim = chrono.Simulation()


world = chrono.ChronoScene()


ground = chrono.ChronoBody()
ground.SetName('Ground')
ground.SetPos(chrono.Vec3(0,0,0))
world.Add(ground)


beam = chrono.ChronoBody()
beam.SetName('Beam')
beam.SetPos(chrono.Vec3(0,10,0))
beam.SetMass(1.0)
world.Add(beam)


joint = chrono.ChronoJointLink2D(beam, ground)
joint.SetPos(chrono.Vec3(0,10,0))
joint.SetAx(chrono.Vec3(0,1,0))
world.Add(joint)


cable = chrono.ChronoANCFElement()
cable.SetName('Cable')
cable.SetBody(beam)
cable.SetPoints([chrono.Vec3(0,10,0), chrono.Vec3(1,12,0)])
cable.SetRestLength(1.0)
cable.SetDamping(0.0)
cable.SetStiffness(1000.0)
world.Add(cable)


sim.NewtonIntegrator.SetTimestep(0.0001)
sim.SetMaxStepSize(0.01)


g = chrono.ChronoVector3(0, -9.81, 0)
world.AddForce(g)


visual = irrlicht.IrrlichtApp()
visual.CreateWindow(irrlicht.E_DRIVER_TYPE_OPENGL, 1024, 768)
visual.AddCamera()
visual.AddLight()
visual.AddScene(world)
visual.Run()


while sim.Step() == True:
    visual.Render()


chrono Finals()