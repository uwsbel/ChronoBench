import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import pychrono.fea as fea
import math as m


# Create the system
system = chrono.ChSystemNSC()

# This is the point on the beam to be hinged to the ground
hinge_loc = chrono.ChVector3d(-1, 0, 0)

# This is the length of the beam
beam_length = 10

# This is the number of elements in the beam
num_elements = 10

# This is the radius of the beam
beam_radius = 0.1

# Element properties
element_length = beam_length / num_elements
element_properties = cascade.ElementProperties(
    0.1, 0.1, 0.1, 0.01, 1000)

# Create the beam, a body with multiple ANCF cable elements
beam = cascade.ChCascadeBodyEasy(element_length,
                                  num_elements, element_properties, 1000)

# Add the visual assets for the beam
vis_params = cascade.AssetParams()
vis_params.type = cascade.VisualizationType_NONE
vis_params.texture = chrono.GetChronoDataFile(
    "cascade/texture/hatch.png")
vis_params.color = cascade.Color(0, 1, 0, 1)
beam.setAllAssets(vis_params)
# vis_params = cascade.AssetParams()
# vis_params.type = cascade.VisualizationType_PRIMITIVES
# vis_params.color = cascade.Color(1, 0, 0, 0)
# beam.setAsset(0, vis_params)

# Set the initial position of the first node of the beam
first_node = beam.GetNode(0)
first_node.SetPos(chrono.ChVector3d(-5, 0, 0))
first_node.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

# Fix the first node to act as a hinge
first_node.SetFixed(True)

# Generate the beam
beam.Initialize()

# Add a point light at the origin
beam.AddPointLight(chrono.ChVector3d(0, 0, 0), 3, 3, 3)

# Add the beam to the system
system.Add(beam)

# Create a large cube as a floor in the scene
floor = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -1, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("cascade/texture/concrete.jpg"))
system.Add(floor)

# Set the gravitational acceleration
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

# Create the Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ANCF Cable Beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.GetSceneManager().getActiveCamera().setTarget(chronirr.vector3dfCH(beam.GetNode(0).GetPos()))
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)

# This simulation setup captures the deformation of the beam under gravity.