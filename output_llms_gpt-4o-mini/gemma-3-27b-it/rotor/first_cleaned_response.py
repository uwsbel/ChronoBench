import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


beam_length = 1.0  
beam_height = 0.1  
beam_width = 0.05  
density = 7850  
youngs_modulus = 2.0e11  
poisson_ratio = 0.3

flywheel_radius = 0.05  
flywheel_mass = 0.1  
flywheel_moment_inertia = 0.5 * flywheel_mass * flywheel_radius**2


motor_torque = 1.0  
motor_speed = 10.0  


mesh = fea.ChMesh()
section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(beam_width, beam_height)
section.SetYoungModulus(youngs_modulus)
section.SetPoissonRatio(poisson_ratio)
section.SetShearModulus(youngs_modulus / (2 * (1 + poisson_ratio)))  
section.SetRayleighDamping(0.01)


node1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
node2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0)))
mesh.AddNode(node1)
mesh.AddNode(node2)


element = fea.ChElementBeamEuler()
element.SetNodes(node1, node2)
element.SetSection(section)
mesh.AddElement(element)


beam_body = fea.ChBodyEasyMesh(mesh)
beam_body.SetMass(density * beam_width * beam_height * beam_length)
beam_body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
sys.Add(beam_body)


flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVector3d(flywheel_moment_inertia, flywheel_moment_inertia, flywheel_moment_inertia))
flywheel.SetPos(chrono.ChVector3d(beam_length / 2, 0, 0))
flywheel.EnableCollision(True)
sys.Add(flywheel)


link_flywheel = chrono.ChLinkLockRevolute()
link_flywheel.Initialize(beam_body, flywheel, chrono.ChCoordsysD(chrono.ChVector3d(beam_length / 2, 0, 0), chrono.QUNIT))
sys.Add(link_flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_body, None, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(motor_speed))
motor.SetTorqueFunction(chrono.ChFunction_Const(motor_torque))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 2, -3))
vis.AddTypicalLights()


vis.AddFEMVisualization(beam_body, True)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)