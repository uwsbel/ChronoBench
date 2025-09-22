import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, 0))








beam_length = 1.0  
beam_radius = 0.02  
beam_density = 7800  
beam_E = 2.1e11  
beam_area = chrono.CH_C_PI * beam_radius**2
beam_inertia = (chrono.CH_C_PI * beam_radius**4) / 4  





beam = chrono.ChBody()
beam.SetName("Beam")
beam.SetMass(beam_density * beam_area * beam_length)
beam.SetInertia(chrono.ChMatrix33D(beam_inertia, 0, 0,
                                   0, beam_inertia, 0,
                                   0, 0, beam_inertia))
system.Add(beam)


beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.Q_from_Euler123(0, 0, 0))
beam.SetBodyFixed(False)


flywheel_radius = 0.05  
flywheel_mass = 0.5  

flywheel = chrono.ChBody()
flywheel.SetName("Flywheel")
flywheel.SetMass(flywheel_mass)
flywheel_inertia = 0.5 * flywheel_mass * flywheel_radius**2
flywheel.SetInertia(chrono.ChMatrix33D(flywheel_inertia, 0, 0,
                                       0, flywheel_inertia, 0,
                                       0, 0, flywheel_inertia))

flywheel_pos = chrono.ChVectorD(beam_length / 2, 0, 0)
flywheel.SetPos(flywheel_pos)
system.Add(flywheel)


fix_left = chrono.ChLinkMateFix()
fix_left.Initialize(beam, system.GetGround(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(fix_left)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(beam, flywheel, chrono.ChFrameD(flywheel_pos))
system.Add(revolute_joint)



motor = chrono.ChLinkMotorRotation()
end_pos = chrono.ChVectorD(beam_length, 0, 0)

frameA = chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0))
frameB = chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0))
motor.Initialize(beam, system.GetGround(), frameA)
system.Add(motor)


motor.SetSpeed(100)  



visualization = chrono.ChVisualization()
visualization.SetShape(chrono.shapeutils.CreateCylinder(beam_radius, beam_length))
visualization.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
beam.GetVisualShape().AddChild(visualization)


application = irr.ChIrrApp(system, "Jeffcott Rotor Simulation", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0.2, 0.2, 0.2))
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 1e-4
application.SetTimestep(time_step)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()