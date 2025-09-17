import pychrono as chrono


system = chrono.ChSystemNSC()


vis = chrono.ChVisualSystemIrrlicht(width=800, height=600)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Epicyclic Gears")
vis.AddCamera(chrono.ChCamera(), chrono.ChVectorD(0, 0, 5))
vis.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))
vis.SetSunLight(chrono.ChColor(0.8, 0.8, 0.8), chrono.ChVectorD(1, 1, 1))


truss = chrono.ChBodyEasyBox(1, 1, 1, 100)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(truss)


bar = chrono.ChBodyEasyCylinder(0.1, 0.5, 100)
bar.SetPos(chrono.ChVectorD(0, 0, 0.5))
bar.SetRot(chrono.Q_from_AngX(chrono.deg2rad(45)))
system.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(0.2, 0.1, 100)
gear1.SetPos(chrono.ChVectorD(0.3, 0, 0.5))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.2, 0.1, 100)
gear2.SetPos(chrono.ChVectorD(0.3, 0, 0.5))
system.Add(gear2)


motor = chrono.ChGearMotor()
motor.SetPos(chrono.ChVectorD(0.3, 0, 0.5))
motor.SetSpeed(chrono.rad2deg(10))  
system.Add(motor)


gear1.AddGear(gear2, 1, 1)  
motor.AddGear(gear1, 1, 1)  


gear1.SetMaterial(chrono.ChMaterialSurface(chrono.ChColor(0.8, 0.2, 0.2), chrono.ChColor(0.5, 0.1, 0.1)))
gear2.SetMaterial(chrono.ChMaterialSurface(chrono.ChColor(0.2, 0.8, 0.2), chrono.ChColor(0.1, 0.5, 0.1)))
bar.SetMaterial(chrono.ChMaterialSurface(chrono.ChColor(0.2, 0.2, 0.8), chrono.ChColor(0.1, 0.1, 0.5)))
truss.SetMaterial(chrono.ChMaterialSurface(chrono.ChColor(0.8, 0.8, 0.8), chrono.ChColor(0.6, 0.6, 0.6)))


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.DoStepDynamics(0.01)


vis.Run()