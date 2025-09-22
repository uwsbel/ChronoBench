import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')
my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


beam = chrono.ChBody()
beam.SetPos(chrono.ChVectorD(0, 10, 0))
beam.SetMass(1)
beam.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
my_system.Add(beam)


num_elements = 10
for i in range(num_elements):
    element = chrono.ChLinkANCF()
    element.Initialize(beam, ground, False, chrono.ChVectorD(0, -10 - i, 0), chrono.ChVectorD(0, -10 - i - 1, 0))
    my_system.Add(element)


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


app = chronoirr.ChIrrApp(my_system, 'Beam deformation', chronoirr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalCamera(chronoirr.vector3df(0, 15, -30))
app.AddTypicalLights()


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()

    
    for i in range(num_elements + 1):
        print(f'Node {i}: {beam.GetPos() + chrono.ChVectorD(0, -10 - i, 0)}')