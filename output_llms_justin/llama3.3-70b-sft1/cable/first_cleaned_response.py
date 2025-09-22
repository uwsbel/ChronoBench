import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import pychrono.fea as fea
import math as m



system = chrono.ChSystemNSC()


hinge_loc = chrono.ChVector3d(-1, 0, 0)


beam_length = 10


num_elements = 10


beam_radius = 0.1


element_length = beam_length / num_elements
element_properties = cascade.ElementProperties(
    0.1, 0.1, 0.1, 0.01, 1000)


beam = cascade.ChCascadeBodyEasy(element_length,
                                  num_elements, element_properties, 1000)


vis_params = cascade.AssetParams()
vis_params.type = cascade.VisualizationType_NONE
vis_params.texture = chrono.GetChronoDataFile(
    "cascade/texture/hatch.png")
vis_params.color = cascade.Color(0, 1, 0, 1)
beam.setAllAssets(vis_params)






first_node = beam.GetNode(0)
first_node.SetPos(chrono.ChVector3d(-5, 0, 0))
first_node.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))


first_node.SetFixed(True)


beam.Initialize()


beam.AddPointLight(chrono.ChVector3d(0, 0, 0), 3, 3, 3)


system.Add(beam)


floor = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -1, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("cascade/texture/concrete.jpg"))
system.Add(floor)


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ANCF Cable Beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.GetSceneManager().getActiveCamera().setTarget(chronirr.vector3dfCH(beam.GetNode(0).GetPos()))
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)