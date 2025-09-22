import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine
import pychrono.fea as chronofea


chrono.SetChronoDataPath('path/to/chrono/data')  


system = chrono.ChSystemNSC()


time_step = 0.01
system.SetTimestepperType(chronoengine.ChTimestepperType.HHT)
system.SetMaxPenetrationRecoverySpeed(1.0)
system.SetMaxIterationsPenetrationRecovery(100)


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Epicyclic Gears Simulation')
visualization.SetTryFullscreen(False)
visualization.SetUseShadows(True)
visualization.SetDefaultFontScale(0.8)
visualization.SetDefaultDirLightDirection(chrono.ChVectorD(0, 1, -1))


visualization.AttachSystem(system)


truss = chrono.ChBodyEasyBox(1, 1, 0.1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0.5))
truss.SetBodyFixed(True)
system.Add(truss)


bar = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 1))
system.Add(bar)


gear1 = chrono.ChBodyEasyBox(0.2, 0.2, 0.05, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.5, 0, 1))
gear1.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(gear1)

gear2 = chrono.ChBodyEasyBox(0.2, 0.2, 0.05, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(-0.5, 0, 1))
gear2.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(gear2)


gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(gear1, truss, chrono.ChVectorD(0, 0, 1))
gear_motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.ChVectorD(0, 0, 10)))
system.Add(gear_motor)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.6)
material.SetRestitution(0.2)
material.SetCompliance(0.00001)
material.SetComplianceT(0.00001)


truss.GetMaterialSurface().SetMaterialProperties(material)
bar.GetMaterialSurface().SetMaterialProperties(material)
gear1.GetMaterialSurface().SetMaterialProperties(material)
gear2.GetMaterialSurface().SetMaterialProperties(material)


run_simulation = True
while run_simulation:
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()

    system.DoStepDynamics(time_step)

    chrono.ChIrrAppTimerStep()

    if visualization.GetDevice().isWindowActive() == False:
        run_simulation = False