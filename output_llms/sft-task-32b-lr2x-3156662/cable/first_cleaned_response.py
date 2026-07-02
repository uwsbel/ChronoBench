import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


mesh = fea.ChMesh()
system.Add(mesh)


material = fea.ChMaterialBeamEuler()
material.SetYoungModulus(2.1e11)  
material.SetGshearModulus(8.1e10)  
material.SetDensity(7800)  


num_elements = 20
beam_length = 1.0
node_spacing = beam_length / num_elements
nodes = []
for i in range(num_elements + 1):
    x = i * node_spacing
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(0.05, 0.02, material)  
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(section)
    mesh.AddElement(element)


fixed_ground = chrono.ChBody()
fixed_ground.SetFixed(True)
system.AddBody(fixed_ground)
hinge_constraint = chrono.ChLinkPointFrame()
hinge_constraint.Initialize(nodes[0], fixed_ground, chrono.ChFramed(nodes[0].GetPos(), nodes[0].GetRot()))
system.AddLink(hinge_constraint)


fea_visual = fea.ChVisualShapeFEA(mesh)
fea_visual.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MF)
fea_visual.SetColorscaleMinMax(-0.1, 0.1)
fea_visual.SetSmoothFaces(True)
mesh.AddVisualShape(fea_visual)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ANCF Cable Beam Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step)