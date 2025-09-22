import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


beam_length = 5.0
beam_diameter = 0.1
num_elements = 10
element_length = beam_length / num_elements
density = 1000  
gravity = chrono.ChVectorD(0, -9.81, 0)


beam = chrono.ChBody()
beam.SetMass(density * chrono.CH_C_PI * (beam_diameter / 2)**2 * beam_length)
beam.SetInertiaXX(chrono.ChVectorD(1, 1, 1))  
beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetPos_dt(chrono.ChVectorD(0, 0, 0))
system.Add(beam)


cable_elements = []
for i in range(num_elements):
    pos_start = chrono.ChVectorD(0, 0, 0) + chrono.ChVectorD(0, i * element_length, 0)
    pos_end = chrono.ChVectorD(0, 0, 0) + chrono.ChVectorD(0, (i + 1) * element_length, 0)
    
    cable = chrono.ChBodyEasyCylinder(beam_diameter / 2, element_length, density, True, True)
    cable.SetPos((pos_start + pos_end) / 2)
    cable.SetRot(chrono.Q_from_AngZ(0))
    cable_elements.append(cable)
    system.Add(cable)


hinge = chrono.ChLinkLock()
hinge.Initialize(ground, beam, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0)))
system.Add(hinge)


system.Set_G_acc(gravity)


application = chronoirr.ChIrrApp(system, "ANCF Beam Simulation", chronoirr.dimension2d(800, 600))
application.AddSkyBox()
application.AddCamera(chronoirr.vector3df(0, 2, -10), chronoirr.vector3df(0, 1, 0))
application.AddLight(chronoirr.vector3df(0, 10, -10), chronoirr.SColorf(1, 1, 1))


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    system.DoStepDynamics(0.01)

    
    for cable in cable_elements:
        application.DrawLine(cable.GetPos(), cable.GetPos() + chrono.ChVectorD(0, element_length, 0), chronoirr.SColor(255, 0, 0))

    application.EndScene()