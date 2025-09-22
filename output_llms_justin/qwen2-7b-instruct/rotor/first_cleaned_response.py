import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess


chrono.Ch纪年().Init()


dt = 0.001
t_end = 10.0


vis_sys = chronoirr.ChVisualSystemIrrlicht()
vis_sys.SetWindowSize(800, 600)
vis_sys.SetWindowTitle('Jeffcott Rotor Simulation')


chrono.Ch纪年().AddSystem(vis_sys)



beam = chrono.ChBeamModel()
beam.SetBeamModelType(chrono.ChBeamModelType.IGA)
beam.SetBeamLength(1.0)
beam.SetBeamCrossSection(0.1, 0.1)
beam.SetBeamMaterial(chrono.ChMaterialSurfaceWelded())
beam.SetBeamDensity(7800.0)
beam.SetBeamMeshDensity(10)
beam.SetBeamIGAOrder(3)
beam.SetBeamIGARefine(1)


flywheel = chrono.ChFlywheel()
flywheel.SetMass(100.0)
flywheel.SetInertiaXX(chrono.ChVectorD(10.0, 10.0, 10.0))


beam.AddBody(flywheel)


motor = chrono.ChMotor()
motor.SetMaxTorque(1000.0)
motor.SetSpeed(100.0)
motor.SetPos(chrono.ChVectorD(0.0, 0.0, 0.0))


beam.AddMotor(motor)


beam.SetBeamPos(chrono.ChVectorD(0.0, 0.0, 0.0))
beam.SetBeamRot(chrono.ChQuaternionD(1.0, 0.0, 0.0, 0.0))
beam.SetBeamForce(chrono.ChVectorD(0.0, 0.0, 0.0))


chrono.Ch纪年().SetTimestepperType(chrono.ChTimestepperType.LAST)
chrono.Ch纪年().SetTimestep(dt)


chrono.Ch纪年().AddModel(beam)


chrono.Ch纪年().Step(t_end)


post_vis = postprocess.ChPostProcessing()
post_vis.AddBeamModel(beam)
post_vis.AddIrrlichtSystem(vis_sys)
post_vis.AddText("Jeffcott Rotor Simulation")
post_vis.AddPlotter(postprocess.ChPlotterLineXY("Angular Velocity", "t", "omega", 0.0, t_end, 0.0, 100.0))
post_vis.AddPlotter(postprocess.ChPlotterLineXY("Angular Acceleration", "t", "alpha", 0.0, t_end, -100.0, 100.0))
post_vis.AddPlotter(postprocess.ChPlotterLineXY("Torque", "t", "tau", 0.0, t_end, -1000.0, 1000.0))
post_vis.AddPlotter(postprocess.ChPlotterLineXY("Flywheel Angular Velocity", "t", "omega_f", 0.0, t_end, -100.0, 100.0))
post_vis.AddPlotter(postprocess.ChPlotterLineXY("Flywheel Angular Acceleration", "t", "alpha_f", 0.0, t_end, -100.0, 100.0))


post_vis.Run()


chrono.Ch纪年().Finalize()