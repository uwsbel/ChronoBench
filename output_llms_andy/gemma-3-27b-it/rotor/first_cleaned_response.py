import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea





my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






beam_length = 1.0
beam_radius = 0.02
num_segments = 20
young_modulus = 2.0e11  
poisson_ratio = 0.3
density = 7850.0


beam = fea.ChIGABeamSection()
beam.Initialize(beam_radius, beam_radius)  
beam.SetYoungModulus(young_modulus)
beam.SetPoissonRatio(poisson_ratio)
beam.SetDensity(density)


iga_element = fea.ChIGAElement()
iga_element.Initialize(beam, num_segments)


iga_mesh = fea.ChIGAMesh()
iga_mesh.AddElement(iga_element)


iga_body = fea.ChIGABody()
iga_body.Initialize(iga_mesh)
iga_body.SetMass(density * chrono.ChVectorD(beam_length, beam_radius * 2, beam_radius * 2).Length())  
iga_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
my_system.Add(iga_body)






flywheel_radius = 0.05
flywheel_mass = 0.1
flywheel_inertia = 0.5 * flywheel_mass * flywheel_radius * flywheel_radius

flywheel = chrono.ChBodyEasy()
flywheel.SetBodyFixed(False)
flywheel.SetMass(flywheel_mass)
flywheel.SetInertia(chrono.ChVectorD(flywheel_inertia, flywheel_inertia, flywheel_inertia))
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0.5, 0))  
flywheel.SetCollide(True)
my_system.Add(flywheel)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(iga_body, flywheel, chrono.ChCoordsys(chrono.ChVectorD(beam_length / 2, 0.5, 0)))
my_system.Add(revolute_joint)






motor_torque = 1.0
motor = chrono.ChMotorLinearSpring()
motor.Set_SpringRestLength(0)
motor.Set_SpringK(0)
motor.Set_SpringR(0)
motor.Set_Torque(motor_torque)
motor.Set_MotorMode(chrono.ChMotorLinearSpring.MOTOR_MODE_VELOCITY)
motor.Set_Velocity(10)  
motor.Initialize(iga_body, chrono.ChCoordsys(chrono.ChVectorD(0, 0.5, 0)))
my_system.Add(motor)





my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
my_system.SetStepTime(0.001)
my_system.SetMaxIterationSteps(100)
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor - IGA Beam')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -2))
vis.AddTypicalLights()





while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    my_system.DoStepDynamics()