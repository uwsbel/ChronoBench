importchrono
importchrono_IGA
importchrono_postprocess
importnumpy as np
importpyrr


system = chrono.ChSystemNSC()
solver = chrono.ChLCPGSolver()
system.SetSolver(solver)
application = chrono.ChIrrApp(system, "Jeffcott Rotor Simulation", chronoirr.dimension2D(800, 600))


application.SetCameraPosition(pyrr.Vector3([0, 2, 2]))
application.SetCameraRotation(pyrr.Vector3([60, 0, 0]))
application.AddTypicalLights()


beam = chrono.IgaChIgaBeamElement()
beam.SetYoungModulus(2.1e11)  
beam.SetPoissonRatio(0.3)     
beam.SetArea(0.01)            
beam.SetMoment(0.0001)        
beam.SetLength(1.0)           
beam.SetNumElements(10)        
beam.SetNumControlPoints(4)    


system.Add(beam)


flywheel = chrono.IgaChIgaFlywheel()
flywheel.SetMass(1.0)          
flywheel.SetRadius(0.1)        
flywheel.SetPosition(pyrr.Vector3([0, 0, 0]))  


ujt = chrono.ChUJT()
ujt.Initialize(beam.GetNode(5), flywheel, pyrr.Matrix33.identity())
system.Add(ujt)


motor = chrono.ChMotorRotation()
motor.SetTorque(10.0)          
motor.SetTargetVelocity(pyrr.Vector3([0, 0, 10]))  
motor.Initialize(beam.GetNode(0), None, pyrr.Matrix33.identity())
system.Add(motor)


gravity = chrono.ChGravity()
gravity.SetGravity(pyrr.Vector3([0, -9.81, 0]))
system.Add(gravity)


beam.SetFEMVisualization(True)
beam.SetFEMVisualizationScale(0.1)


damping = chrono.ChDamping()
damping.SetDampingFactors(0.1, 0.1)
system.Add(damping)


application.AssetBindAll()
application.AssetUpdateAll()
application.AddAllTypicalLights()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()