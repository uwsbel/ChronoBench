import chrono as ch
import chrono.irrlicht as chirr
import numpy as np


ch.set_default_G_acc(0, 0, -9.81)  


sys = ch.ChSystemSMC()


app = chirr.IrrlichtApplication(sys)
app.setTimestep(0.01)
app.set_window_size(ch.ChVector2(800, 600))
app.set_caption('Folding Tablecloth Simulation')
app.show_stats()


mat = ch.ChMaterialShellKirchhoffSMC()
mat.SetDensity(1000)  
mat.SetYoungModulus(100e3)  
mat.SetPoissonRatio(0.3)


grid_size = 10
num_nodes = (grid_size + 1) ** 2
num_elements = grid_size ** 2

nodes = []
elements = []


for i in range(grid_size + 1):
    for j in range(grid_size + 1):
        node = ch.ChNodeFEAxyzP(
            ch.ChVector3(i - grid_size // 2, j - grid_size // 2, 0),
            ch.ChQuaternion(1, 0, 0, 0)
        )
        sys.AddNode(node)
        nodes.append(node)


for i in range(grid_size):
    for j in range(grid_size):
        n1 = nodes[i * (grid_size + 1) + j]
        n2 = nodes[(i + 1) * (grid_size + 1) + j]
        n3 = nodes[(i + 1) * (grid_size + 1) + (j + 1)]
        n4 = nodes[i * (grid_size + 1) + (j + 1)]

        elem = ch.ChElementShellReissner4(
            n1, n2, n3, n4,
            mat,
            ch.ChVector3(0, 0, 1)
        )
        sys.AddElement(elem)
        elements.append(elem)


sys.SetSolverType(ch.ChSolver.Type.PARDISO_MKL)


floor_body = ch.ChBodyEasyBody(
    ch.ChCollisionShapeBox(100, 100, 0.1),
    1000
)
floor_body.SetPos(ch.ChVector3(0, 0, -5))
sys.AddBody(floor_body)


contact_mat = ch.ChContactMaterialSMC()
contact_mat.SetFriction(0.5)
contact_mat.SetRestitutionCoeff(0.5)


floor_shape = ch.ChCollisionShapeBox(
    ch.ChVector3(100, 100, 0.1),
    ch.ChVector3(0, 0, -5)
)
floor_shape.SetContactMaterial(contact_mat)
floor_body.AddCollisionShape(floor_shape)


tablecloth_shape = ch.ChCollisionShapeTriangleMesh(
    ch.ChTriangleMeshConnected(),
    contact_mat
)


mesh = ch.ChTriangleMeshConnected()
for i, elem in enumerate(elements):
    n1, n2, n3, n4 = elem.GetNodesXYZP()
    tri1 = ch.ChTriangle(n1, n2, n3)
    tri2 = ch.ChTriangle(n1, n3, n4)
    mesh.AddTriangle(tri1)
    mesh.AddTriangle(tri2)

tablecloth_shape.SetTriangleMesh(mesh)
for elem in elements:
    n1, n2, n3, n4 = elem.GetNodesXYZP()
    tablecloth_shape.AddTriangle(
        ch.ChTriangle(n1, n2, n3),
        ch.ChVector3(0, 0, 0)
    )
    tablecloth_shape.AddTriangle(
        ch.ChTriangle(n1, n3, n4),
        ch.ChVector3(0, 0, 0)
    )


for node in nodes:
    node.AddCollisionShape(tablecloth_shape)


while app.run():
    sys.DoStepDynamics(0.01)