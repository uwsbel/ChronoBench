import chrono
from chrono import ChMaterial, ChSystem, ChBody, ChShape, ChLinkMate, ChVisualization, ChShapes, ChMatrixDynamic, ChFrame
from chrono.irrlicht import ChirrApp
from chrono.fea import ChElementShellQT
from chrono.fea.shell import ChShellInterface
from chrono.fea.shell.driver import ChDriverShellLin
from chrono.fea.truss import ChElementT3
from chrono.fea.truss import ChTrussInterface
from chrono.core import ChTimer
from chrono.collision import ChCollisionModel
from chrono.irrlicht.user_interface import ChIrrGui


ChSystem.SetSolverType(ChSystem.SOLVER_PARDISO_MKL)
my_system = ChSystem()



tablecloth_width = 1.5  
tablecloth_length = 2.0  
num_elem_width = 10
num_elem_length = 15

tablecloth_nodes = []
tablecloth_elements = []
tablecloth_material = ChMaterial.KirchhoffIsotropic(50, 0.3)

for i in range(num_elem_width + 1):
    for j in range(num_elem_length + 1):
        x = i * tablecloth_width / num_elem_width
        y = j * tablecloth_length / num_elem_length

        if i < num_elem_width and j < num_elem_length:
            
            node = ChBody()
            node.SetPos(chrono.ChVectorD(x, y, 0.01))
            node.SetMass(0.0)
            node.SetCollide(False)
            my_system.AddBody(node)
            tablecloth_nodes.append(node)

            
            if i % 2 == 0 and j % 2 == 0:
                el = ChElementShellQT(node, tablecloth_material, 0.1, 0.1)
            else:
                el = ChElementShellQT(node, tablecloth_material, 0.1, 0.1)
                el.SetThickness(0.005)

            el.SetPressure(0.0)
            el.SetBendingStiffness(1000)
            el.SetShearStiffness(1000)
            el.SetTorsionalStiffness(1000)
            el.SetFlexuralStiffness(1000)

            my_system.AddElement(el)
            tablecloth_elements.append(el)


for node in tablecloth_nodes:
    node.SetPos(node.GetPos() + chrono.ChVectorD(0, 0, 0.01))


my_vis = ChVisualization()
my_vis.AddSystem(my_system)
my_vis.SetWindowSize(1280, 720)
my_vis.SetTimestep(1e-4)
my_vis.SetShowPhysics(True)
my_vis.SetShowCollision(True)
my_app = ChirrApp(my_vis)


my_gui = ChIrrGui()
my_app.AddUserInterface(my_gui)


timer = ChTimer()
timer.Start()

while my_app.GetDevice().run():
    my_app.BeginScene()
    my_vis.BeginScene()

    my_system.DoStepDynamics(my_vis.GetTimestep())

    my_vis.DrawAll()
    my_app.DrawAll()

    my_vis.EndScene()
    my_app.EndScene()

my_app.GetDevice().drop()

timer.Stop()
print("Elapsed time: ", timer.GetElapsedTime())