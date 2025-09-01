import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


system = chrono.ChSystemNSC()


solver = chrono.ChSolverMINRES()
system.SetSolver(solver)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
alpha = -0.2
system.SetTimestepperHHTAlpha(alpha)


length_beam = 1.0
radius_beam = 0.05
density_beam = 1000.0
young_modulus = 2e11
poisson_ratio = 0.3

beam = fea.ChBeamIGA()
beam.SetNumSections(4)
beam.SetNumSpans(4)
beam.SetNodeSpacing(length_beam / 4)
beam.SetMaterial(young_modulus, poisson_ratio)
beam.SetDensity(density_beam)
beam.SetRadius(radius_beam)
beam.SetInitialLength(length_beam)
beam.Initialize()


system.Add(beam)


radius_flywheel = 0.1
thickness_flywheel = 0.05
density_flywheel = 7800.0
inertia_flywheel = chrono.ChVectorD(0.5 * density_flywheel * chrono.ChMath.Pi * radius_flywheel**4,
                                    0.5 * density_flywheel * chrono.ChMath.Pi * radius_flywheel**4,
                                    0.5 * density_flywheel * chrono.ChMath.Pi * radius_flywheel**4 * thickness_flywheel**2)

flywheel = chrono.ChBodyEasyCylinder(radius_flywheel, thickness_flywheel, density_flywheel, True, True)
flywheel.SetPos(chrono.ChVectorD(0.5 * length_beam, 0, 0))
flywheel.SetInertiaXX(inertia_flywheel)


system.Add(flywheel)


link_flywheel = chrono.ChLinkMateFix()
link_flywheel.Initialize(flywheel, beam)
system.Add(link_flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetNodeA(), beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor_function = chrono.ChFunction_Const(chrono.CH_C_PI)  
motor.SetMotionFunction(motor_function)
system.Add(motor)


application = chronoirr.ChIrrApp(system, "Jeffcott Rotor Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddTypicalCamera(chronoirr.vector3df(0, -2, 0), chronoirr.vector3df(0, 0, 0))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()