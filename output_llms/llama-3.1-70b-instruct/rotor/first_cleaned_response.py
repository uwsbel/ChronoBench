import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
mysystem.SetStep(0.001)  


beam_length = 1.0
beam_radius = 0.05
beam_density = 7850
beam_young_modulus = 2.1e11
beam_poisson_ratio = 0.3
beam_mesh_size = 0.1

beam = fea.ChBeamSectionEuler(beam_length, beam_radius, beam_density, beam_young_modulus, beam_poisson_ratio)
beam.SetMeshSize(beam_mesh_size)


flywheel_mass = 10
flywheel_radius = 0.2
flywheel_thickness = 0.05
flywheel_density = 7850

flywheel = chrono.ChBodyEasyCylinder(flywheel_mass, flywheel_radius, flywheel_thickness, flywheel_density)


motor_torque = 10
motor_speed = 100

motor = chrono.ChLinkMotorRotationSpeed()
motor.SetTorque(motor_torque)
motor.SetSpeed(motor_speed)


beam.Add(flywheel, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.Add(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


mysystem.Add(beam)
mysystem.Add(flywheel)
mysystem.Add(motor)


irr = chronoirr.ChVisualSystemIrrlicht()
irr.SetWindowSize(800, 600)
irr.SetWindowTitle("Jeffcott Rotor Simulation")
irr.SetLogoFile("chrono_logo.png")
irr.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


fem_vis = fea.ChVisualizationFEAmesh(mysystem)
fem_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODAL_FIELD)
fem_vis.SetFEMfieldFunction(fea.ChVisualizationFEAmesh.F_NODAL_DISPLACEMENT)
fem_vis.SetColoring(1, 1, 1, 1)
fem_vis.SetWireframe(false)


while mysystem.GetChTime() < 10:
    mysystem.DoStepDynamics(mysystem.GetStep())
    irr.BeginScene()
    irr.DrawAll()
    fem_vis.Render()
    irr.EndScene()
    irr.Screenshot("screenshot_%04d.png" % int(mysystem.GetChTime()))