fromchrono = __import__('chrono')
fromchronoimportchrono as chrono
frompychronoimportchrono as pychrono
frompychronoimportirrlichtengine
frompychronoimportchronoengine
frompychronoimportvisual
frompychronoimportutils


pychrono.init()


world = pychrono.ChSystemNSC()


default_gravity = pychrono.ChVectorD(0, -9.81, 0)


ground = pychrono.ChBodyEasyBox(1000, 1, 100, 10000)
ground.SetPos(pychrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
world.Add(ground)


beam_length = 1.0
beam_width = 0.1
beam_height = 0.05


beam = pychrono.ChLinkCable()
beam.SetCableType(chrono.ChCableType.ANCF)
beam.SetBeamLength(beam_length)
beam.SetBeamWidth(beam_width)
beam.SetBeamHeight(beam_height)
beam.SetBeamMaterial(chrono.ChMaterialSurfaceNSC())
beam.SetBeamDensity(7800)
beam.SetBeamCrossSection(chrono.ChVectorD(beam_width / 2, beam_height / 2, beam_height / 2))
beam.SetBeamYoungModulus(200e9)
beam.SetBeamPoissonRatio(0.3)
beam.SetBeamDensity(7800)
beam.SetBeamDensity(7800)
beam.SetBeamMassDensity(7800)
beam.SetBeamMassPerLength(beam_density * beam_width * beam_height)


beam_nodes = [pychrono.ChVectorD(0, 0, 0), pychrono.ChVectorD(beam_length, 0, 0)]
beam_connects = [(0, 1)]


for node in beam_nodes:
    beam_node = pychrono.ChBodyEasySphere(0.01, 1000, False, False)
    beam_node.SetPos(node)
    beam_node.SetBodyFixed(True)
    world.Add(beam_node)


for i in range(len(beam_connects)):
    node1 = beam_connects[i][0]
    node2 = beam_connects[i][1]
    beam.AddSegment(beam_nodes[node1], beam_nodes[node2], beam)


beam.AddForce(default_gravity)


renderer = pychrono.IrrlichtRenderer()
renderer.SetCameraPosition(pychrono.ChVectorD(0, 10, 10))
renderer.SetCameraTarget(pychrono.ChVectorD(0, 0, 0))
renderer.AddTypicalLogo()
renderer.AddTypicalSky()
renderer.AddTypicalLights()
renderer.AddTypicalFloor()
renderer.AddTypicalCameraControls()


world.SetTimestepperType(chrono.ChTimestepper::FixedTimeStep)
world.SetTimestep(0.001)
world.SetMaxItersSolver(1000)

for step in range(10000):
    world.DoOneStepDynamics()
    renderer.RenderStep(world)

pychrono.finalize()