import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import math


chrono.SetChairsThreading(chrono.GetNumProcs())
chrono.SetChronoDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()


beam_length = 1.0
beam_radius = 0.05
beam_thickness = 0.01
beam_E = 2.1e9
beam_nu = 0.3
beam_density = 7850
beam_ang_vel = 0
beam_ang_acc = 0
beam_mass = math.pi * beam_radius ** 2 * beam_thickness * beam_density
beam_Jxx = 0.5 * beam_mass * beam_radius ** 2
beam_Jyy = 0.25 * beam_mass * beam_radius ** 2 + beam_mass * beam_length ** 2 / 12
beam_Jzz = beam_Jyy

beam_section = fea.ChBeamSectionCosseratTimoshenko()
beam_section.SetThickness(beam_thickness)
beam_section.SetYoungModulus(beam_E)
beam_section.SetShearModulus(beam_E / (2 * (1 + beam_nu)))
beam_section.SetDensity(beam_density)
beam_section.SetRayleighDamping(0.05)
beam_section.SetMaterialColor(chrono.ChColor(1, 0.5, 0.5))

beam = fea.ChBeamIGA(beam_section, beam_length, 20, 3)
beam.SetPos_mm(chrono.ChVector3d(0, 0, 0))
beam.SetRot_mm(chrono.ChQuaterniond(1, 0, 0, 0))
beam.SetSectionalLength(beam_length)
beam.SetSectionalRadius(beam_radius)
beam.SetSectionalMass(beam_mass)
beam.SetSectionalInertiaY(beam_Jyy)
beam.SetSectionalInertiaZ(beam_Jzz)
beam.SetSectionalInertiaYZ(0)
beam.SetSectionalArea(chrono.ChVector3d(math.pi * beam_radius ** 2, 0, 0))
beam.SetSectionalCentroid(chrono.ChVector3d(0, 0, 0))
beam.SetSectionalFrame(chrono.ChMatrix33d(1, 0, 0, 0, 1, 0, 0, 0, 1))
beam.SetSectionalAngularVelocity(chrono.ChVector3d(beam_ang_vel, beam_ang_vel, beam_ang_vel))
beam.SetSectionalAngularAcceleration(chrono.ChVector3d(beam_ang_acc, beam_ang_acc, beam_ang_acc))

system.Add(beam.GetBodyA())
system.Add(beam.GetBodyB())
system.Add(beam.GetBodySection(0))


flywheel_mass = 1
flywheel_radius = 0.1
flywheel_inertia = chrono.ChMatrix33d(flywheel_mass * flywheel_radius ** 2, 0, 0, 0, 0.5 * flywheel_mass * flywheel_radius ** 2, 0, 0, 0, 0.5 * flywheel_mass * flywheel_radius ** 2)

flywheel_mat = chrono.ChContactMaterialNSC()
flywheel_mat.SetFriction(0.3)
flywheel_mat.SetRestitution(0.2)

flywheel_body = chrono.ChBody()
flywheel_body.SetMass(flywheel_mass)
flywheel_body.SetInertia(flywheel_inertia)
flywheel_body.SetPos(chrono.ChVector3d(0, 0, beam_length / 2))
flywheel_body.SetContactMaterial(flywheel_mat)
system.Add(flywheel_body)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetBodyA(), flywheel_body, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(0.5))
motor.SetChronoConstraintType(chrono.ChConstraint.ChConstraintTypeMOTOR)
system.Add(motor.GetConstraint())


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle('IGA Jeffcott Rotor')
vis.SetWindowSize(800, 600)
vis.SetRenderMode(irr.RenderMode.RT_TrackMarks)
vis.Initialize()


for i in range(beam.GetNsection()):
    section = beam.GetBodySection(i)
    section.AddVisualizationBeamSection(0.005)


vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.6, -0.4, 0.4), chrono.ChVector3d(0, 0, 0))
vis.AddLightDirectional()
vis.AddLightPoint(chrono.ChVector3d(0.5, 0.5, 0.5), chrono.ChVector3d(0, 0, 0), 5, 0.2, 0.2)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)