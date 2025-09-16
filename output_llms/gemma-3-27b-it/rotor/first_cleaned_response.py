import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess


chrono.SetChronoDataPath("./")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



beam_length = 1.0
beam_radius = 0.02
beam_segments = 20
beam_order = 3  


beam = chrono.ChBodyEasy.New()
beam.SetBodyFixed(False)
beam.SetMass(1.0)
beam.SetInertia(1.0)


section = chrono.ChNurbsSection()
section.SetShapeType(chrono.ChNurbsShape.CIRCLE)
section.SetRadius(beam_radius)
section.SetSegments(beam_segments)


iga_beam = chrono.ChIGABeam.New()
iga_beam.SetSection(section)
iga_beam.SetLength(beam_length)
iga_beam.SetOrder(beam_order)
iga_beam.BuildMesh()


beam.AddAsset(iga_beam)
beam.SetCollide(True)
beam.SetVisualizationType(chrono.ChVisualizationType.MESH)


system.Add(beam)


flywheel_radius = 0.05
flywheel_mass = 0.5
flywheel = chrono.ChBodyEasy.New()
flywheel.SetBodyFixed(False)
flywheel.SetMass(flywheel_mass)
flywheel.SetInertia(chrono.ChVectorD(flywheel_mass * flywheel_radius * flywheel_radius,
                                     flywheel_mass * flywheel_radius * flywheel_radius,
                                     flywheel_mass * flywheel_radius * flywheel_radius))
flywheel.SetShape(chrono.ChSphereShape(flywheel_radius))
flywheel.SetCollide(True)
flywheel.SetVisualizationType(chrono.ChVisualizationType.MESH)


flywheel_pos = chrono.ChVectorD(beam_length / 2.0, 0, 0)
flywheel.SetPos(flywheel_pos)


system.Add(flywheel)


joint = chrono.ChLinkRevolute.New()
joint.Initialize(beam, flywheel, chrono.ChCoordsys(flywheel_pos))
system.Add(joint)


motor_torque = 1.0
motor = chrono.ChMotorLinearSpring.New()
motor.Set_SpringRestLength(0.0)
motor.Set_SpringK(0.0)
motor.Set_SpringR(0.0)
motor.Set_Torque(motor_torque)
motor.Set_Mode(chrono.ChMotorMode.TORQUE_CONTROL)


motor.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(motor)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()


time_step = 0.001
time_end = 10.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > time_end:
        vis.GetIrrlichtApplication().Close()