import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea







mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
mysystem.Add(ground)








beam_length = 1.0
beam_radius = 0.05
beam_material = fea.ChMaterialFEA(rho=7850, E=200e9, nu=0.3)


knots = [0, 0, 0.25, 0.5, 0.75, 1, 1, 1]
control_points = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(beam_length / 4, 0, 0),
    chrono.ChVectorD(beam_length / 2, 0, 0),
    chrono.ChVectorD(3 * beam_length / 4, 0, 0),
    chrono.ChVectorD(beam_length, 0, 0),
]


beam = fea.ChBeamIGA(beam_material, knots, control_points, beam_radius)
beam.SetNumElements(10)


mysystem.Add(beam)








flywheel_radius = 0.2
flywheel_mass = 10


flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_radius, flywheel_mass)


flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
chrono.ChLinkMateFix(flywheel, beam, chrono.ChCoordsysD(chrono.ChVectorD(beam_length / 2, 0, 0)))
mysystem.Add(flywheel)








motor = chrono.ChBody()
motor.SetBodyFixed(True)
motor.SetPos(chrono.ChVectorD(0, 0, 0))
mysystem.Add(motor)


motor_constraint = chrono.ChLinkMotorRotationSpeed(motor, beam, chrono.ChVectorD(1, 0, 0))
motor_constraint.SetSpeedFunction(chrono.ChFunction_Const(10))
mysystem.Add(motor_constraint)








vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()


fea.ChVisualizationFEA(vis, beam)


while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.Render()