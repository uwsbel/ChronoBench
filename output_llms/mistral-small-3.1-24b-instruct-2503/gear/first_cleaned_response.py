import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine
import pychrono.fea as chronofea
import pychrono.vehicle as chronovehicle


chrono.SetChronoDataPath('../data/')  


system = chrono.ChSystemNSC()


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Epicyclic Gear System')
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))


truss = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)


rotating_bar = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000, True, True)
rotating_bar.SetPos(chrono.ChVectorD(0, 0, 0.5))
system.Add(rotating_bar)


gear1 = chrono.ChBodyEasyCylinder(0.5, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.5, 0, 0.5))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.5, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(-0.5, 0, 0.5))
system.Add(gear2)


gear_motor = chrono.ChFunction_Const(chrono.ChVectorD(0, 0, 1))
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(rotating_bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(gear_motor)
motor.SetSpeed(chrono.ChVectorD(0, 0, 1))  
system.Add(motor)


material = chronoirr.ChVisualMaterial()
material.SetAmbientColor(chrono.ChColor(0.2, 0.2, 0.2))
material.SetDiffuseColor(chrono.ChColor(0.8, 0.8, 0.8))
material.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
material.SetReflectance(0.5)

truss.GetVisualShape(0).SetMaterial(material)
rotating_bar.GetVisualShape(0).SetMaterial(material)
gear1.GetVisualShape(0).SetMaterial(material)
gear2.GetVisualShape(0).SetMaterial(material)


system.SetSolverType(chrono.ChSolver.BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)


run_visualization = True
while run_visualization:
    visualization.Render()
    visualization.Update()
    system.DoStepDynamics(0.01)

    
    if visualization.GetDevice().isWindowActive() == False:
        run_visualization = False


visualization.Close()