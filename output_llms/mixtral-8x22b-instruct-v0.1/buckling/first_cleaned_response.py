import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

my_application = chronoirr.ChIrrApp(chronoirr.ChIrrApp.Type.CHRONO, "Beam Buckling Simulation", chronoirr.dimension2du(800, 600))


my_system = chrono.ChSystemNSC()

beam_length = 1.0
beam_width = 0.05
beam_height = 0.05
beam_density = 7850
beam_youngs_modulus = 200e9
beam_poisson_ratio = 0.3


beam = chrono.ChElementBeamFEA()
beam.SetLength(beam_length)
beam.SetWidth(beam_width)
beam.SetHeight(beam_height)
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_youngs_modulus)
beam.SetPoissonRatio(beam_poisson_ratio)

my_system.Add(beam)


def custom_motor_function(t):

    pass


motor = chrono.ChLinkMotorRotationSpeed()
motor.Set_speed_function(custom_motor_function)


my_system.Add(motor)


fixed_point = chrono.ChVectorD(0, 0, 0)
beam_end = chrono.ChVectorD(beam_length, 0, 0)
constraint = chrono.ChLinkLockPointPlanar()
constraint.Initialize(beam.GetNodeN(), fixed_point, beam.GetNodeN(), beam_end)


my_system.Add(constraint)


beam.GetNodeN().SetPos(chrono.ChVectorD(0, 0, 0))
beam.GetNodeN().SetRot(chrono.ChQuaternionD(1, 0, 0, 0))

my_system.SetIntegrationType(chrono.ChSystem.IntegrationType.HHT)
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity


timestepper = chrono.ChTimestepperHHT()
timestepper.SetMaxiters(100)
timestepper.SetTol(1e-10)
timestepper.SetVerbose(False)


my_system.SetTimestepper(timestepper)


my_application.AddTypicalLogo(chronoirr.GetChronoDataFile("logo_pychrono_alpha.png"))
my_application.AddTypicalSky(chronoirr.GetChronoDataFile("skybox/sky.jpg"))
my_application.AddTypicalFloor(chronoirr.GetChronoDataFile("bluetile.jpg"))
my_application.AddLightWithShadow(chronoirr.GetChronoDataFile("bluetile.jpg"), 10, 20, -10, 10, 10, 10)


beam_visualization = chronoirr.ChIrrNode(my_application.GetSceneManager(), my_application.GetVideoDriver())
beam_visualization.SetPos(beam.GetNodeN().GetPos())
beam_visualization.SetRot(beam.GetNodeN().GetRot())
beam_visualization.SetScale(chrono.ChVectorD(beam_length, beam_width, beam_height))
beam_visualization.SetBody(beam)
my_application.GetSceneManager().GetRootSceneNode().AddChild(beam_visualization)


my_application.SetTimestep(0.01)
my_application.SetTryRealtime(True)
my_application.Run()