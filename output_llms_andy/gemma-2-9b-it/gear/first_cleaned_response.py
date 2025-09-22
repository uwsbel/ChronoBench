import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("data/")  
system = chrono.ChSystemNSC()


truss = chrono.ChBodyEasyBox(1.0, 0.5, 0.2, 1000.0)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(truss)


bar = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000.0)
bar.SetPos(chrono.ChVectorD(0, 0, 0.3))
bar.SetBodyFixed(False)
system.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(0.1, 0.1, 1000.0)
gear1.SetPos(chrono.ChVectorD(0.2, 0, 0.5))
gear1.SetBodyFixed(False)
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.1, 0.1, 1000.0)
gear2.SetPos(chrono.ChVectorD(0.4, 0, 0.5))
gear2.SetBodyFixed(False)
system.Add(gear2)


gear1_mesh = chrono.ChMeshSphere(0.1)
gear1.AddAsset(gear1_mesh)
gear2_mesh = chrono.ChMeshSphere(0.1)
gear2.AddAsset(gear2_mesh)


gear_joint = chrono.ChLinkLockUniversal(gear1, gear2)
gear_joint.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(gear_joint)


motor = chrono.ChMotorRotation(chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0.5), chrono.Q_from_Euler123(0, 0, 0)), gear1, chrono.ChVectorD(0, 0, 1), 100)
system.Add(motor)


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.Set_Step_Size(0.01)
system.Set_Max_Substeps(10)


vis.InitVisualization()
vis.Add(truss)
vis.Add(bar)
vis.Add(gear1)
vis.Add(gear2)
vis.SetCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))


for i in range(1000):
    system.DoStepDynamics()
    vis.Render()

vis.CloseVisualization()