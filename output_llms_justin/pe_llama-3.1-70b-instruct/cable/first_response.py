import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.mkb as mkb

# Set path to Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system and visualization
sys = chrono.ChSystemNSC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Simulation")
vis.SetLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create the ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetBodyFixed(True)
sys.Add(ground)

# Create the beam
beam_length = 10
beam_width = 0.5
beam_height = 0.5
beam_density = 1000
beam_E = 1e9
beam_G = 0.3 * beam_E
beam_section = mkb.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(beam_width, beam_height)
beam_section.SetYoungModulus(beam_E)
beam_section.SetShearModulus(beam_G)
beam_section.SetRayleighDamping(0.000)

beam_mesh = mkb.ChMesh()
beam_node1 = mkb.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
beam_node2 = mkb.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0)))
beam_mesh.AddNode(beam_node1)
beam_mesh.AddNode(beam_node2)
beam_element = mkb.ChElementBeamEuler()
beam_element.SetNodes(beam_node1, beam_node2)
beam_element.SetSection(beam_section)
beam_mesh.AddElement(beam_element)

beam_body = chrono.ChBody()
beam_body.SetPos(chrono.ChVector3d(0, 0, 0))
beam_body.SetMass(beam_density * beam_length * beam_width * beam_height)
beam_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
beam_body.AddVisualShape(chrono.ChVisualShapeMesh(beam_mesh))
sys.Add(beam_body)

# Create the hinge joint
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(ground, beam_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(hinge)

# Set up the simulation loop
stepping = chrono.ChTimestepperHMM()
sys.SetTimestepper(stepping)
tstep = 0.01
tend = 10
vis.AttachSystem(sys)

while vis.Run():
    sys.DoStepDynamics(tstep)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    # Print the beam's nodal positions
    print("Beam Nodal Positions:")
    for node in beam_mesh.GetNodes():
        print(node.GetPos().x, node.GetPos().y, node.GetPos().z)