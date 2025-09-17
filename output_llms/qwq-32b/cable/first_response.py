import pychrono as chrono
import pychrono.irrlicht as irr

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

# Create ANCF material properties
material = chrono.ChMaterialANCF()
material.SetYoungModulus(1e6)  # Young's modulus (Pa)
material.SetPoissionsRatio(0.3)
material.SetDensity(7850)  # Density (kg/m³)

# Create the ANCF beam
beam = chrono.fea.ChBeamANCF()
beam.SetNodesNumber(21)       # 20 elements, 21 nodes
beam.SetTotalLength(10.0)     # Length along x-axis (meters)
beam.SetSectionWidth(0.01)    # Cross-section width (meters)
beam.SetSectionHeight(0.01)   # Cross-section height (meters)
beam.SetMaterial(material)

# Add the beam to the system
system.Add(beam)

# Fix the first node to the ground (origin)
link = chrono.ChLinkLockPointFrame()
link.Initialize(beam.GetNodes()[0], system)
system.Add(link)

# Visualization setup
visualizer = irr.ChIrrApp(system, 'ANCF Beam Simulation', irr.dimension2du(800, 600))
visualizer.AddTypicalLights()
visualizer.AddTypicalCamera(chrono.ChVectorD(0, -10, 5), chrono.ChVectorD(0, 0, 0))
visualizer.AssetSetMargin(0)

# Configure beam visualization
beam.SetVisualization(chrono.fea.ChVisualizationBeam())
beam.AddAsset(chrono.ChColorAsset(0.8, 0, 0))  # Red color for the beam

# Add a ground marker at the origin
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().AddBox(0.5, 0.5, 0.5)  # Small cube for visualization
ground.SetCollide(False)
ground.AddAsset(chrono.ChColorAsset(0.3, 0.3, 0.3))  # Gray color
visualizer.Add(ground)

# Initialize and run the visualization
visualizer.SetTimestep(0.005)
visualizer.Initialize()
visualizer.SetCameraPosition(chrono.ChVectorD(0, -10, 5))
visualizer.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))
visualizer.MainLoop()