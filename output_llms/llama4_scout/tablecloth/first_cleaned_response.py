import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


app = irr.IrrlichtApplication()


step_size = 0.01
frame_rate = 60


solver = ch.ChSolverPardisoMKL()


E = 100000  
nu = 0.3    
rho = 1000  


material = ch.ChKirchhoffMaterial(E, nu, rho)


nx = 20
ny = 20
Lx = 2.0  
Ly = 2.0  


mesh = ch.ChMesh()


for i in range(nx + 1):
    for j in range(ny + 1):
        x = Lx * i / nx
        y = Ly * j / ny
        z = 0.0
        node = ch.ChNodexyz(x, y, z)
        mesh.AddNode(node)


for i in range(nx):
    for j in range(ny):
        node1 = mesh.GetNode(i * (ny + 1) + j)
        node2 = mesh.GetNode((i + 1) * (ny + 1) + j)
        node3 = mesh.GetNode((i + 1) * (ny + 1) + j + 1)
        node4 = mesh.GetNode(i * (ny + 1) + j + 1)

        
        element = ch.ChElementShellKirchhoff(material, [node1, node2, node3, node4])
        mesh.AddElement(element)


for i in range(nx + 1):
    node = mesh.GetNode(i * (ny + 1))
    node.SetFixed(True)


folding_force = ch.ChForce()
folding_force.SetForce(1000, 0, 0)  
mesh.GetNode(nx // 2 * (ny + 1) + ny // 2).AddForce(folding_force)


system = ch.ChSystem()


system.Add(mesh)


system.SetSolver(solver)


vis = irr.IrrlichtVis()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetTitle("Folding Tablecloth Simulation")


while vis.Run() and not app.HasToClose():
    system.DoStepDynamics(step_size)


app.Close()

import chrono as ch
import chrono.irrlicht as irr

def main():
    
    ch.ChEngine.Initialize()

    
    app = irr.IrrlichtApplication()

    
    step_size = 0.01
    frame_rate = 60

    
    solver = ch.ChSolverPardisoMKL()

    
    E = 100000  
    nu = 0.3    
    rho = 1000  

    
    material = ch.ChKirchhoffMaterial(E, nu, rho)

    
    nx = 20
    ny = 20
    Lx = 2.0  
    Ly = 2.0  

    
    mesh = ch.ChMesh()

    
    for i in range(nx + 1):
        for j in range(ny + 1):
            x = Lx * i / nx
            y = Ly * j / ny
            z = 0.0
            node = ch.ChNodexyz(x, y, z)
            mesh.AddNode(node)

    
    for i in range(nx):
        for j in range(ny):
            node1 = mesh.GetNode(i * (ny + 1) + j)
            node2 = mesh.GetNode((i + 1) * (ny + 1) + j)
            node3 = mesh.GetNode((i + 1) * (ny + 1) + j + 1)
            node4 = mesh.GetNode(i * (ny + 1) + j + 1)

            
            element = ch.ChElementShellKirchhoff(material, [node1, node2, node3, node4])
            mesh.AddElement(element)

    
    for i in range(nx + 1):
        node = mesh.GetNode(i * (ny + 1))
        node.SetFixed(True)

    
    folding_force = ch.ChForce()
    folding_force.SetForce(1000, 0, 0)  
    mesh.GetNode(nx // 2 * (ny + 1) + ny // 2).AddForce(folding_force)

    
    system = ch.ChSystem()

    
    system.Add(mesh)

    
    system.SetSolver(solver)

    
    vis = irr.IrrlichtVis()
    vis.AttachSystem(system)
    vis.SetWindowSize(800, 600)
    vis.SetTitle("Folding Tablecloth Simulation")

    
    while vis.Run() and not app.HasToClose():
        system.DoStepDynamics(step_size)

    
    app.Close()

if __name__ == "__main__":
    main()