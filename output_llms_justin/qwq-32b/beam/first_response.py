import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# Create the FEA system
sys = fea.ChSystemFEM()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

# Create nodes along the x-axis from (0,0,0) to (5,0,0) with 1m spacing
nodes = []
for i in range(6):
    node = fea.ChNodeFEAxyz()
    node.SetPos(chrono.ChVectorD(i, 0, 0))
    sys.AddNode(node)
    nodes.append(node)

# Fix the first node to simulate a cantilever
nodes[0].SetFixed(True)

# Create beam elements between consecutive nodes
for i in range(len(nodes) - 1):
    beam = fea.ChLinkFEAbeam()
    beam.Initialize(nodes[i], nodes[i+1])
    
    # Set beam properties
    beam.Set_A(0.1)        # Cross-sectional area (m²)
    beam.Set_Iy(0.01)      # Moment of inertia (y-axis)
    beam.Set_Iz(0.01)      # Moment of inertia (z-axis)
    beam.Set_E(2e11)       # Young's modulus (Pa)
    beam.Set_nu(0.3)       # Poisson's ratio
    
    # Add visualization asset for the beam
    vis = fea.ChVisualizationFEAbeam()
    vis.SetRadius(0.1)     # Beam radius
    beam.AddAsset(vis)
    
    sys.AddLink(beam)

# Visualization setup using Irrlicht
renderer = irr.ChIrrApp(sys, 'Beam FEA Simulation', irr.dimension2du(800, 600))
renderer.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
renderer.SetCameraVertical(chrono.CH_VECT_Y)
renderer.AllocateResources()

# Add sphere visualization to nodes
for node in nodes:
    sphere = irr.ChSphereShape()
    sphere.GetSphereGeometry().rad = 0.2  # Sphere radius
    node.AddAsset(sphere)

# Run the simulation
renderer.SetTimestep(0.001)
renderer.SetTryRealtime(True)
renderer.Execute()