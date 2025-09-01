import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()


system = chrono.ChSystemSMC()


node_mass = 1.0  
node_inertia = chrono.ChVectorD(0.0, 0.0, 0.0)  
beam_section = chrono.ChBeamSectionAdvanced()
beam_section.SetDensity(1e-9)  
beam_section.SetYoungModulus(2e5)  
beam_section.SetGshearModulus(7.5e4)  
beam_section.SetBeamRaleyghDamping(0.01)


node_A = chrono.ChNodeFEAxyzD(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
node_A.SetMass(node_mass)
node_A.SetInertia(node_inertia)
system.Add(node_A)

node_B = chrono.ChNodeFEAxyzD(chrono.ChFrameD(chrono.ChVectorD(100, 0, 0)))
node_B.SetMass(node_mass)
node_B.SetInertia(node_inertia)
system.Add(node_B)


beam = chrono.ChElementBeamEuler()
beam.SetNodes(node_A, node_B)
beam.SetSection(beam_section)
system.Add(beam)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


force_magnitude = 100  
force_direction = chrono.ChVectorD(0, 0, 1)
node_B.SetForce(force_magnitude * force_direction)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Finite Elements Simulation")
vis.Initialize()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(0.01)