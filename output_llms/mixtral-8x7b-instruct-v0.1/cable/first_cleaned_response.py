from pychrono.core import *
from pychrono.irrlicht import *
from pychrono.fea import *


my_chrono = ChronoEngine()
my_chrono.Set_G_Acceleration(Vector3(0, -9.81, 0))


my_sim = ChronoSimulation(my_chrono)






beam_body = ChBeamANCF(
    my_chrono,
    ChCoordsysD(VectorD(0, 0, 0), QUNIT),
    ChVectorD(1, 0.1, 0.1),
    10,
    2,
    0.01,
    ChMatrix33(CUMATRIX33),
)


system = my_chrono.GetSystem()
system.AddBody(beam_body)






hinge_joint = ChLinkLockRevolute(
    system.Get_Body_Weldable(beam_body.GetID()),
    ChFrameD(beam_body.GetFrame_A()),
    Vector3(0, 0, 1),
)
system.AddLink(hinge_joint)






vis = IrrlichtVisualization(my_chrono, "Beam Simulation")
vis.AddLogo(ChVectorD(0, 0, 0))
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(ChVectorD(0, 5, -10), ChVectorD(0, 0, 0))


vis.Add(beam_body, ChColor(0.5, 0.5, 0.5))





my_sim.Set_Delta_Time(0.001)
my_sim.Set_Time_Stepping(ChTimeSteppingType::RungeKutta4)
my_sim.Set_Visualization(vis)


for i in range(int(1e4)):
    my_sim.DoStepDynamics()
    vis.WaitKey(5)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


my_sim.DumpState(True, True)