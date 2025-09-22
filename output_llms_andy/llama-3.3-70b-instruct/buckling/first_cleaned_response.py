import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam = fea.ChBeamSectionEuler()
beam.SetDensity(7850)
beam.SetYoungModulus(2e11)
beam.SetPoissonRatio(0.3)
beam.SetIyy(1e-5)
beam.SetIzz(1e-5)
beam.SetJ(1e-6)


mesh = fea.ChMesh()
num_elements = 10
length = 10
for i in range(num_elements):
    node1 = chrono.ChNodeFEAxyz(chrono.ChVectorD(i * length / num_elements, 0, 0))
    node2 = chrono.ChNodeFEAxyz(chrono.ChVectorD((i + 1) * length / num_elements, 0, 0))
    element = fea.ChElementBeamEuler(beam, node1, node2)
    mesh.Add(element)


sys.Add(mesh)


def motor_function(time):
    return 10 * np.sin(time)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(ground, mesh.GetNode(0))
motor.SetMotorFunction(motor_function)
sys.Add(motor)


constraint = chrono.ChLinkPointPoint()
constraint.Initialize(ground, mesh.GetNode(0), chrono.ChVectorD(0, 0, 0))
sys.Add(constraint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()


solver = chrono.ChSolverNSC()
timestepper = chrono.ChTimestepperEuler()


sys.SetSolver(solver)
sys.SetTimestepper(timestepper)


time_step = 0.01
end_time = 10
while sys.GetChTime() < end_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()


vis.Close()