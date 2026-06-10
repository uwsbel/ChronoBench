import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr




sys = chrono.ChSystemSMC()                     
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))   


dt = 0.005                                      
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)




sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL)




mesh = fea.ChMesh()
sys.Add(mesh)




mat = fea.ChMaterialShellIsotropic(1e7, 0.3, 0.001)




width  = 2.0    
length = 2.0    
nx     = 20     
ny     = 20     


nodes = []
for i in range(nx + 1):
    for j in range(ny + 1):
        x = i * width / nx
        y = j * length / ny
        node = fea.ChNodeFEAxyz(chrono.ChVectorD(x, y, 0.0))
        node.SetFixed(False)          
        mesh.AddNode(node)
        nodes.append(node)


for i in range(nx):
    for j in range(ny):
        
        n0 = nodes[i * (ny + 1) + j]
        n1 = nodes[(i + 1) * (ny + 1) + j]
        n2 = nodes[(i + 1) * (ny + 1) + (j + 1)]
        n3 = nodes[i * (ny + 1) + (j + 1)]

        elem = fea.ChElementShellKirchhoff()
        elem.SetMaterial(mat)
        elem.SetNodes(n0, n1, n2, n3)
        mesh.AddElement(elem)




tol = 1e-4   
for node in nodes:
    p = node.GetPos()
    if (abs(p.x)          < tol and abs(p.y)          < tol) or \
       (abs(p.x - width)  < tol and abs(p.y)          < tol) or \
       (abs(p.x)          < tol and abs(p.y - length) < tol) or \
       (abs(p.x - width)  < tol and abs(p.y - length) < tol):
        node.SetFixed(True)




centre_node = None
for node in nodes:
    p = node.GetPos()
    if abs(p.x - width / 2) < tol and abs(p.y - length / 2) < tol:
        centre_node = node
        break

if centre_node:
    force = chrono.ChForce()
    force.SetMod(5.0)                                 
    force.SetDir(chrono.ChVectorD(0, 0, -1))         
    centre_node.AddForce(force)




app = irr.ChIrrApp(sys,
                    "Table‑cloth folding (Kirchhoff shells)",
                    irr.dimension2d_u32(1280, 720))

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(irr.vector3df(2.0, 2.0, 3.0),
                     irr.vector3df(1.0, 1.0, 0.0))
app.AddTypicalLights()


app.AddMesh(irr.ChIrrNodeMeshAdapter(mesh))




app.SetStep(dt)          
app.SetFPS(30)            

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    app.DoStep()