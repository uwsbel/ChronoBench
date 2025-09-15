import chrono as ch
import chrono.fea as fea
import chrono.irrlicht as irr


system = ch.ChSystemNSC()
system.Set_G_acc(ch.ChVectorD(0, 0, 0))


solver = ch.ChSolverMINRES()
system.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-12)
solver.EnableDiagonalPreconditioner(True)


mesh = fea.ChMesh()
system.Add(mesh)


E = 2e11    
nu = 0.3    
rho = 7800  
damping = 0.01
beam_material = fea.ChMaterialBeamEuler(E, E/(2*(1+nu)), nu, rho)


section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(0.1, 0.02)
section.SetMaterial(beam_material)
section.SetRayleighDamping(damping)


num_elements = 10
length = 5.0
node_spacing = length / num_elements
nodes = []

for i in range(num_elements + 1):
    pos = ch.ChVectorD(i * node_spacing, 0, 0)
    node = fea.ChNodeFEAxyzrot()
    node.SetPos(pos)
    node.SetRot(ch.ChQuaternionD(1, 0, 0, 0))
    if i == 0:
        node.SetFixed(True)  
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    beam = fea.ChElementBeamEuler()
    beam.SetNodes(nodes[i], nodes[i+1])
    beam.SetSection(section)
    mesh.AddElement(beam)


force = ch.ChVectorD(0, -1000, 0)  
load_container = ch.ChLoadContainer()
load = fea.ChLoadNodeForce(nodes[-1], force)
load_container.Add(load)
system.Add(load_container)


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_BEAM)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
vis_mesh.SetBeamResolution(5)
vis_mesh.SetBeamWidth(0.02)
mesh.AddAsset(vis_mesh)


application = irr.ChIrrApp(system, "Beam FEM Demonstration", irr.dimension2du(1280, 720))
application.AddTypicalCamera(irr.vector3df(5, 3, 0), irr.vector3df(2.5, 0, 0))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.001
application.SetTimestep(time_step)
application.SetTryRealtime(True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStepDynamics(time_step)
    application.EndScene()