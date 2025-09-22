import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()




cloth_length = 1.0   
cloth_width = 1.0    
n_x = 20             
n_y = 20             
dx = cloth_length / (n_x - 1)
dy = cloth_width / (n_y - 1)
cloth_thickness = 0.002  


mesh = fea.ChMesh()


rho = 500           
E = 2e7             
nu = 0.3            
material = fea.ChMaterialShellKirchhoffIsothropic(E, nu, rho, cloth_thickness)


nodes = []
for iy in range(n_y):
    row = []
    for ix in range(n_x):
        xpos = ix * dx - cloth_length/2
        ypos = iy * dy - cloth_width/2
        zpos = 0.1  
        node = fea.ChNodeFEAxyzD(chrono.ChVectorD(xpos, ypos, zpos), chrono.ChVectorD(0, 0, 1))
        node.SetMass(0.25 * rho * dx * dy * cloth_thickness)
        mesh.AddNode(node)
        row.append(node)
    nodes.append(row)


for iy in range(n_y - 1):
    for ix in range(n_x - 1):
        node00 = nodes[iy][ix]
        node01 = nodes[iy][ix+1]
        node11 = nodes[iy+1][ix+1]
        node10 = nodes[iy+1][ix]
        element = fea.ChElementShellKirchhoff_4()
        element.SetNodes(node00, node01, node11, node10)
        element.SetMaterial(material)
        element.SetThickness(cloth_thickness)
        mesh.AddElement(element)


for ix in range(n_x):
    node = nodes[0][ix]
    node.SetFixed(True)


fold_force = chrono.ChVectorD(0, 0, -10)  
for ix in range(n_x):
    node = nodes[-1][ix]
    node.SetForce(fold_force)

system.Add(mesh)


application = chronoirr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(1024, 768)
application.SetWindowTitle('PyChrono Tablecloth Folding Simulation')
application.Initialize()
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, -1.5, 0.7), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()


vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMglyphType(fea.ChVisualizationFEAmesh.GLYPH_NODE_DOT_POS)
vis_mesh.SetFEMdataType(fea.ChVisualizationFEAmesh.DATA_NONE)
vis_mesh.SetColorscaleMinMax(-0.01, 0.01)
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(True)
mesh.AddAsset(vis_mesh)


solver = chrono.ChSolverPardisoMKL()
system.SetSolver(solver)


application.AssetBindAll()
application.AssetUpdateAll()
application.AddShadowAll()

timestep = 0.002
while application.Run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(timestep)
    application.EndScene()