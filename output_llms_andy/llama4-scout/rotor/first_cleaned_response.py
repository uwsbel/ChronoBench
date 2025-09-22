import chrono as ch
import chrono.irrlicht as irr
import chrono.postprocess as post


ch.ChGlobalSettings.SetChronoVerbose(False)


sys = ch.ChSystemNSC()


solver = ch.ChSolverPSAI()
solver.SetTolerance(1e-4)
sys.SetSolver(solver)


integrator = ch.ChTimestepperEulerImplicitLinearized()
integrator.SetMaxIters(200)
sys.SetIntegrator(integrator)




beam = ch.ChBeamIGA()
beam.SetPos_mm(ch.ChVector3d(0, 0, 0))
beam.SetRotQ(ch.ChQuaterniond(1, 0, 0, 0))
beam.SetLength(1.0)  
beam.SetSectionWidth(0.05)  
beam.SetSectionHeight(0.05)  
beam.SetNumElements(10)  
beam.SetMaterial(ch.ChBeamMaterialSection(0.785e9, 0.3e9, 7850))  
sys.Add(beam)


flywheel = ch.ChBody()
flywheel.SetPos_mm(beam.GetPos_mm() + ch.ChVector3d(0.5, 0, 0))
flywheel.SetMass(10)  
flywheel.SetInertiaXX(ch.ChVector3d(1, 1, 1))
sys.Add(flywheel)


joint = ch.ChLinkLockSpherical()
joint.Init(beam, flywheel, beam.GetPos_mm(), flywheel.GetPos_mm())
sys.Add(joint)


motor = ch.ChLinkMotorRotation()
motor.Init(beam, ch.ChBodyFixed(), ch.ChFrame3d(beam.GetPos_mm() + ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
motor.SetMotorFunction(ch.ChFunction_Const(0.1))  
sys.Add(motor)




sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))


app = irr.IrrlichtApplication(sys)
app.SetTimestep(0.01)


post.FemVisualization(beam, 0.01)


app.Show()


while app.Run():
    pass