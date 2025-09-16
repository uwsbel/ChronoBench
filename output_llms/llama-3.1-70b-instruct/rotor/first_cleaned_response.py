import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import pychrono.mkl as mkl
import numpy as np


mysim = chrono.ChSystemSMC()


chrono.ChCollisionSystem.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionSystem.SetDefaultSuggestedMargin(0.001)


beam_length = 1.0
beam_radius = 0.05
beam_density = 7850
beam_young_modulus = 2.1e11
beam_poisson_ratio = 0.3
beam_mesh_size = 0.1
beam_visualization_type = fea.PostProcessing.VISUALIZATION_FEM

beam = fea.ChBeamIGA()
beam.SetLength(beam_length)
beam.SetRadius(beam_radius)
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_young_modulus)
beam.SetPoissonRatio(beam_poisson_ratio)
beam.SetMeshSize(beam_mesh_size)
beam.SetVisualizationType(beam_visualization_type)


flywheel_mass = 1.0
flywheel_radius = 0.1
flywheel_thickness = 0.05
flywheel_density = 7850
flywheel_inertia = chrono.ChVectorD(flywheel_mass * (flywheel_radius ** 2) / 2, flywheel_mass * (flywheel_radius ** 2) / 2, flywheel_mass * (flywheel_radius ** 2) / 2)

flywheel = chrono.ChBodyEasyCylinder(flywheel_mass, flywheel_radius, flywheel_thickness, flywheel_density, True, True)
flywheel.SetInertiaXX(flywheel_inertia)
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))


beam.AddBody(flywheel)


motor_torque = 10.0
motor_speed = 10.0

motor = chrono.ChLinkMotorRotationSpeed()
motor.SetTorque(motor_torque)
motor.SetSpeed(motor_speed)


beam.AddLink(motor)


window = irr.ChVisualSystemIrrlicht()
window.SetWindowSize(800, 600)
window.SetWindowTitle("Jeffcott Rotor Simulation")
window.AddTypicalLights()


window.AddCamera(chrono.ChVectorD(0, 0, 1))
window.AddTypicalSky()
window.AddTypicalLogo()
window.AddCamera(chrono.ChVectorD(0, 0, 1))
window.SetShowFPS(True)


window.AddFEMbeamVisualization(beam, beam_visualization_type)


step_size = 0.01
sim_time = 10.0

while window.GetDevice().run():
    mysim.DoStepDynamics(step_size)
    window.BeginScene()
    window.DrawAll()
    window.EndScene()

    if mysim.GetChTime() > sim_time:
        break


window.GetDevice().closeDevice()