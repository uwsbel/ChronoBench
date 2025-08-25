import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.ChSystem.SetNumThreads(4)  


system = chrono.ChSystem()


beam_length = 2.0
beam_height = 0.1
beam_width = 0.1
num_elements = 10


beam_mesh = fea.ChMesh()


material = fea.ChContinuumElastic()
material.Set_E(200e9)  
material.Set_v(0.3)    


for i in range(num_elements):
    
    x = i * (beam_length / num_elements)
    node1 = fea.ChNodeFEAxyz(x, 0, 0)
    node2 = fea.ChNodeFEAxyz(x + (beam_length / num_elements), 0, 0)
    
    
    beam_mesh.AddNode(node1)
    beam_mesh.AddNode(node2)
    
    
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(node1, node2)
    beam_element.SetSection(beam_width, beam_height)
    beam_element.SetMaterial(material)
    
    
    beam_mesh.AddElement(beam_element)


system.Add(beam_mesh)


fixed_node = beam_mesh.GetNode(0)
fixed_body = chrono.ChBody()
fixed_body.SetPos(fixed_node.GetPos())
fixed_body.SetMass(0)  
system.Add(fixed_body)


def apply_force():
    force = chrono.ChForce()
    force.SetMode(chrono.ChForce.FORCE)
    force.SetDir(chrono.ChVectorD(0, -1000, 0))  
    force.SetVpoint(beam_mesh.GetNode(num_elements - 1).GetPos())
    system.Add(force)


solver = chrono.ChSolverPSOR()
system.SetSolver(solver)
system.SetTimestepperType(chrono.ChTimestepper.Type.EULER)


application = chronoirr.ChIrrApp(system, "Beam Buckling Simulation", chronoirr.dimension2d(800, 600))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(1, 1, 5), chrono.ChVectorD(1, 0, 0))
application.AddLight(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(1, 1, 1))


application.SetTimestep(0.01)  
while application.GetDevice().run():
    apply_force()  
    system.DoStepDynamics(application.GetTimestep())
    application.DrawAll()


application.GetDevice().drop()