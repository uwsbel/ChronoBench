import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


beam_length = 5.0
beam_width_y = 0.2
beam_width_z = 0.1
young_modulus = 2.1e11  
shear_modulus = 8.1e10  
rayleigh_damping = 0.001


system = chrono.ChSystemSMC()


mesh = fea.ChMesh()


section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(beam_width_y, beam_width_z)
section.SetYoungModulus(young_modulus)
section.SetShearModulus(shear_modulus)
section.SetRayleighDamping(rayleigh_damping)


node1 = fea.ChNodeFEAxyzrot()
node1.SetPos(chrono.ChVectorD(0, 0, 0))
node2 = fea.ChNodeFEAxyzrot()
node2.SetPos(chrono.ChVectorD(beam_length, 0, 0))
mesh.AddNode(node1)
mesh.AddNode(node2)


element = fea.ChElementBeamEuler()
element.SetNodes(node1, node2)
element.SetSection(section)
mesh.AddElement(element)


system.Add(mesh)


visual_system = chronoirr.ChVisualSystemIrrlicht()
visual_system.AttachSystem(system)
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle('Beam FEA Simulation')
visual_system.Initialize()
visual_system.AddSkyBox()
visual_system.AddCamera(chronoirr.ChVectorD(beam_length/2, 3, 5))
visual_system.AddTypicalLights()


sphere_radius = 0.05
for node in mesh.GetNodes():
    sphere_body = chrono.ChBody()
    sphere_body.SetPos(node.GetPos())
    sphere_body.SetBodyFixed(True)
    sphere_shape = chrono.ChVisualShapeSphere(sphere_radius)
    sphere_shape.SetColor(chrono.ChColor(1, 0, 0))
    sphere_body.AddVisualShape(sphere_shape)
    system.Add(sphere_body)


cylinder_radius = (beam_width_y + beam_width_z)/2
cylinder_body = chrono.ChBody()
cylinder_body.SetPos(chrono.ChVectorD(beam_length/2, 0, 0))
cylinder_body.SetBodyFixed(True)
cylinder_shape = chrono.ChVisualShapeCylinder(cylinder_radius, beam_length)
cylinder_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
cylinder_body.AddVisualShape(cylinder_shape)
system.Add(cylinder_body)


while visual_system.Run():
    system.DoStepDynamics(0.001)
    visual_system.BeginScene()
    visual_system.Render()
    visual_system.EndScene()