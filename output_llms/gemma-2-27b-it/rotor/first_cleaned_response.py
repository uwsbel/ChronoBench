import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


beam_length = 1.0
beam_radius = 0.05
beam_nodes = 10  


knots = [0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0]
control_points = [chrono.ChVectorD(i * beam_length / (beam_nodes - 1), 0, 0) for i in range(beam_nodes)]
nurbs_curve = fea.ChBSplineCurve(knots, control_points, 3)  


beam_section = fea.ChBeamSectionIGA(nurbs_curve, beam_radius)
beam_element = fea.ChElementBeamIGA(beam_section)


flywheel_radius = 0.2
flywheel_mass = 10.0
flywheel_inertia = 0.5 * flywheel_mass * flywheel_radius**2

flywheel = chrono.ChBodyEasyBox(flywheel_radius * 2, flywheel_radius * 2, 0.1, 1000)
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(flywheel_inertia, flywheel_inertia, flywheel_inertia))


constraint = chrono.ChLinkLockRevolute(beam_element.GetNode(beam_nodes // 2), flywheel)
my_system.Add(constraint)


motor_speed = 100.0  

motor = chrono.ChLinkMotorRotationSpeed(beam_element.GetNode(0))
motor.SetTargetSpeed(motor_speed)
my_system.Add(motor)


viz = irr.ChIrrApp(my_system, "Jeffcott Rotor IGA Simulation", irr.dimension2du(800, 600))
viz.AddTypicalSky()
viz.AddTypicalLights()
viz.AddCamera(chrono.ChVectorD(2, 1, 1))


fea.ChVisualizationFEA(my_system)


viz.SetTimestep(0.001)
while viz.GetDevice().run():
    viz.BeginScene()
    viz.Render()
    viz.DoStep()
    viz.EndScene()