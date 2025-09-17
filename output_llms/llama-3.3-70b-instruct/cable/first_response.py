# Import necessary modules
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the PyChrono environment and core components
sys = chrono.ChSystemNSC()

# Set the default parameters
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # gravity

# Create the ground body
ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, chrono.ChVectorD(0, -5, 0))
sys.Add(ground)

# Create the beam
beam = chrono.ChBodyEasyCableANCF(sys, 10, 0.1, 1000, True, chrono.ChVectorD(0, 5, 0))
sys.Add(beam)

# Set the beam's material properties
beam.GetMaterial().SetYoungModulus(2e11)
beam.GetMaterial().SetPoissonsRatio(0.3)
beam.GetMaterial().SetDampingRatio(0.1)

# Create a hinge constraint between the beam and the ground
hinge = chrono.ChLinkMateFixed()
hinge.SetName("hinge")
hinge.Initialize(beam, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
sys.Add(hinge)

# Create the Irrlicht engine for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -20))
vis.AddTypicalLights()

# Simulation loop
time_step = 0.01
end_time = 10
while sys.GetChTime() < end_time:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()

    # Print the beam's deformation and nodal positions
    print("Time: ", sys.GetChTime())
    for i in range(beam.GetNDofN()):
        print("Node {}: {}".format(i, beam.GetNodeN(i).GetPos()))

# Clean up
vis.CloseWindow()