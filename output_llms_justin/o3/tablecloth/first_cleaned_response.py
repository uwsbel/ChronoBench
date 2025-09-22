import pychrono as chrono
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())       
sys = chrono.ChSystemSMC()                                 
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))               


solver = chrono.ChSolverPardisoMKL()
solver LockSparsityPattern = True                          
sys.SetSolver(solver)




mesh = chrono.ChMesh()


E_mod      = 1.0e7        
nu         = 0.3          
rho        = 500          
thickness  = 0.002        

mat = chrono.ChMaterialShellKirchhoff(rho, E_mod, nu)      


Nx   = 30             
Ny   = 30             
Lx   = 0.60           
Ly   = 0.60           

dx   = Lx / Nx
dy   = Ly / Ny


nodes = [[None]*(Ny+1) for _ in range(Nx+1)]


for ix in range(Nx+1):
    for iy in range(Ny+1):
        x = 0.0 + ix*dx
        y = 0.0                                      
        z = 0.0 + iy*dy
        
        if iy == Ny:
            y = -0.05                                

        node = chrono.ChNodeFEAxyzrot(chrono.ChVectorD(x, y, z), chrono.ChQuaternionD(1,0,0,0))
        node.SetMass(rho*thickness*dx*dy)            
        mesh.AddNode(node)
        nodes[ix][iy] = node


for iy in range(Ny+1):
    nodes[0][iy].SetFixed(True)


for ix in range(Nx):
    for iy in range(Ny):
        node00 = nodes[ix  ][iy  ]
        node10 = nodes[ix+1][iy  ]
        node11 = nodes[ix+1][iy+1]
        node01 = nodes[ix  ][iy+1]

        element = chrono.ChElementShellKirchhoff()
        element.SetNodes(node00, node10, node11, node01)
        element.SetMaterial(mat)
        element.SetThickness(thickness)
        element.SetAlphaDamp(0.04)                   
        mesh.AddElement(element)

mesh.SetAutomaticGravity(False)                      
sys.Add(mesh)




app = irr.ChIrrApp(sys, "Kirchhoff Shell Table-cloth Folding", irr.dimension2du(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights(irr.vector3df( 2,  4,  2),
                     irr.vector3df(-2,  4, -2),
                     160.0, 2.0)
app.AddTypicalCamera(irr.vector3df(-0.2, 0.4, 0.8),
                     irr.vector3df( 0.3, 0.0, 0.3))


vis_mesh = chrono.ChVisualizationFEAmesh(mesh)
vis_mesh.SetFEMDataType(chrono.ChVisualizationFEAmesh.E_PLOT_SURFACE)
vis_mesh.SetColorScaleMinMax(0, 1e5)     
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddAsset(vis_mesh)

app.AssetBindAll()
app.AssetUpdateAll()




TIME_STEP = 1e-3
app.SetTimestep(TIME_STEP)

print("Running simulation.  Press Esc in the visualization window to quit.")

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()