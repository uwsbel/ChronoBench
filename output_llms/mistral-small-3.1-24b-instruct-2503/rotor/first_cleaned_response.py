import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem
import pychrono.solver as solver
import pychrono.engine as engine


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetMass(1)
system.AddBody(ground)


beam_material = fem.ChContinuumMaterialElastic()
beam_material.SetE(210e9)  
beam_material.Setv(0.3)    

beam = fem.ChElementBeamIGA()
beam.SetMaterial(beam_material)
beam.SetLength(1.0)  
beam.SetCrossSectionArea(0.01)  
beam.SetCrossSectionMoment(0.0001)  

system.Add(beam)


beam_fixed = chrono.ChLinkLock()
beam_fixed.Initialize(ground, beam)
system.AddLink(beam_fixed)


flywheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 1000, True, True)
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  
system.AddBody(flywheel)


flywheel_link = chrono.ChLinkLock()
flywheel_link.Initialize(beam, flywheel)
system.AddLink(flywheel_link)


rotational_motor = chrono.ChLinkMotorRotationSpeed()
rotational_motor.Initialize(ground, beam, chrono.ChFrameD(beam.GetFrame_REF_to_abs()))
rotational_motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.ChVectorD(0, 0, 1)))  
system.AddLink(rotational_motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


vis.AddElementFEMbeam(beam, 10, chrono.ChColor(1, 0, 0))
vis.AddBodySystem(system)


system.SetSolverType(solver.ChSolverMINRES())
system.SetSolverWarmStarting(False)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-10)

run_time = 10.0  
time_step = 0.01  

current_time = 0.0
while current_time < run_time:
    system.DoStepDynamics(time_step)
    vis.Render()
    current_time += time_step


vis.Close()