import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC.SetNumThreads(1)
system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


beam_length = 5.0  
num_elements = 10   
element_length = beam_length / num_elements
beam_radius = 0.1   


ancf_cable = chrono.ChCable()
ancf_cable.SetDensity(1000)  


for i in range(num_elements + 1):
    node = chrono.ChNodeFEAxyz(chrono.ChVectorD(i * element_length, 0, 0))
    node.SetMass(0.1)  
    ancf_cable.AddNode(node)


for i in range(num_elements):
    element = chrono.ChElementCableANCF()
    element.SetNodes(ancf_cable.GetNode(i), ancf_cable.GetNode(i + 1))
    element.SetDiameter(beam_radius)
    element.SetYoungModulus(2e11)  
    element.SetDensity(1000)  
    system.Add(element)


gravity = chrono.ChForce()
gravity.SetMode(chrono.ChForce.FORCE)
gravity.SetV(chrono.ChVectorD(0, -9.81, 0))
system.Add(gravity)


application = chronoirr.ChIrrApp(system, "ANCF Beam Simulation", chronoirr.dimension2d(800, 600))
application.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono.png")
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 5, 10), chrono.ChVectorD(0, 0, 0))
application.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(1, 1, 1))


while application.GetDevice().run():
    system.DoStepDynamics(0.01)  

    
    application.BeginScene()
    application.DrawAll()

    
    for i in range(num_elements):
        node1 = ancf_cable.GetNode(i)
        node2 = ancf_cable.GetNode(i + 1)
        application.DrawLine(node1.GetPos(), node2.GetPos(), chrono.ChColor(0, 0, 1))

    
    for i in range(num_elements + 1):
        node = ancf_cable.GetNode(i)
        application.DrawPoint(node.GetPos(), chrono.ChColor(1, 0, 0), 0.1)

    application.EndScene()


application.Close()