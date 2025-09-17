importchrono
importchrono.IGA as chrono_iga
importchrono irrlicht as chrono_irr


system = chrono.ChSystemNSC()
solver = chrono.ChIterativeSolverLS()
system.SetSolver(solver)
system.SetMax iteration(100)
system.SetTolerance(1e-6)


application = chrono_irr.ChIrrApp(system, "Jeffcott Rotor Simulation", chrono_irr.dimension2D(800, 600))
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 0.5, 1), chrono.ChVectorD(0, 0, 0))


beam = chrono_iga.ChIgaBeamElement()
beam.SetYoungModulus(2.1e11)  
beam.SetPoissonRatio(0.3)     
beam.SetAreaMoment(0.001)     
beam.SetArea(0.0001)         
beam.SetLength(1.0)          
beam.SetNumElements(10)       
beam.SetNumControlPoints(4)   


system.Add(beam)


flywheel = chrono_iga.ChIgaFlywheel()
flywheel.SetMass(1.0)        
flywheel.SetRadius(0.1)      
flywheel.SetWidth(0.02)      


constraint = chrono.ChUJT()
constraint.Initialize(beam.GetNode(beam.GetNumNodes()//2), flywheel)
system.Add(constraint)
system.Add(flywheel)


motor = chrono.ChMotorRotation()
motor.SetTorque(10.0)        
motor.SetSpeedTarget(10.0)   


motor_constraint = chrono.ChUJT()
motor_constraint.Initialize(beam.GetNode(beam.GetNumNodes()-1), motor)
system.Add(motor_constraint)
system.Add(motor)


gravity = chrono.ChForce()
gravity.SetF(chrono.ChVectorD(0, -9.81, 0))
beam.AddForce(gravity)


beam.SetFEMVisualization(True)
beam.SetFEMVisualizationScale(0.1)


system.SetDamping(0.1)


fixed_constraint = chrono.ChFixed()
fixed_constraint.Initialize(beam.GetNode(0))
system.Add(fixed_constraint)


application.AssetBindAll()
application.AssetUpdateAll()

while application.Run():
    system.DoStepDynamics(0.001)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()