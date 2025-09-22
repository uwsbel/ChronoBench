import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ------------------------------------------------------------------
# 1) Initialize Chrono system
# ------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())   # ensure data path is set
system = chrono.ChSystemSMC()

# choose a more robust linear solver
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.SetTolForce(1e-8)
system.SetMaxPenetrationRecoverySpeed(1.0)

# ------------------------------------------------------------------
# 2) Create an FEA mesh for a straight beam
# ------------------------------------------------------------------
mesh = fea.ChMesh()

# beam geometry + material
beam_length = 1.0
num_elems  = 20
beam_diam  = 0.02
E_modulus  = 2e11
nu         = 0.3
rho_beam   = 7850

# volumetric elastic material for beam
beam_mat = fea.ChContinuumElastic(E_modulus, nu, rho_beam)

# Euler-Bernoulli circular cross section
beam_section = fea.ChBeamSectionEuler(beam_diam, beam_diam, 0, 0, 0, beam_mat)

# create beam nodes
nodes = []
for i in range(num_elems + 1):
    x = beam_length * i / num_elems
    node = fea.ChNodeFEAxyzrot(chrono.ChVectorD(x, 0, 0))
    # fix the leftmost node (built-in clamp)
    if i == 0:
        node.SetFixed(True)
    mesh.AddNode(node)
    nodes.append(node)

# create beam elements
for i in range(num_elems):
    elem = fea.ChElementBeamEuler()
    elem.SetNodes(nodes[i], nodes[i+1])
    elem.SetSection(beam_section)
    # simple spring‐type visualization for element
    elem.SetVisType(fea.ChElement.EVisType.E_VIS_SPRING)
    mesh.AddElement(elem)

# add the mesh to the mechanical system
system.Add(mesh)

# OPTIONAL: add a simple visual asset so that you see the beam
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetDiffuseColor(chrono.ChColor(0.7, 0.2, 0.2))
beam_asset = fea.ChVisualizationFEAmesh(mesh)
beam_asset.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MVONM)
beam_asset.SetColorscaleMinMax(-1.0, 1.0)
beam_asset.SetSmoothFaces(True)
beam_asset.SetWireframe(False)
beam_asset.GetMesh().AddMaterial(vis_mat)
mesh.AddAsset(beam_asset)

# ------------------------------------------------------------------
# 3) Create a rigid "pusher" at the right end and tie it to the last node
# ------------------------------------------------------------------
pusher = chrono.ChBody()
pusher.SetBodyFixed(False)
pusher.SetMass(1.0)
pusher.SetInertiaXX(chrono.ChVectorD(0.01,0.01,0.01))
pusher.SetPos(chrono.ChVectorD(beam_length, 0, 0))
pusher.SetCollide(False)
system.AddBody(pusher)

# tie the last beam node to the pusher
link_point = chrono.ChLinkPointFrame()
link_point.Initialize(nodes[-1], pusher)
system.AddLink(link_point)

# ------------------------------------------------------------------
# 4) Add a linear motor to drive the pusher inward
# ------------------------------------------------------------------
motor = chrono.ChLinkMotorLinearPosition()
motor.SetName("compressor")
# we want to move the pusher along the negative X-axis
# so we define the relative frame in which the motor acts:
motor.Initialize(pusher,           # body A
                 system.Get_bodylist()[0],  # body B = ground (the first body is ground)
                 chrono.ChFrameD(chrono.ChVectorD(0,0,0),
                               chrono.QUNIT))    # motor frame has no rotation
# define a simple ramp: y(t) = y0 + v*t
y0 = 0.0
v  = -0.2   # compress at 0.2 m/s toward negative X
ramp = chrono.ChFunction_Ramp(y0, v)
motor.SetMotionFunction(ramp)
system.Add(motor)

# ------------------------------------------------------------------
# 5) Set up the Irrlicht visualization
# ------------------------------------------------------------------
app = chronoirr.ChIrrApp(
    system, 
    "Beam Buckling with FEA", 
    chronoirr.dimension2du(1024,768)
)
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(beam_length*0.5, -0.5, 0.5))

# bind and update assets
app.AssetBindAll()
app.AssetUpdateAll()

# ------------------------------------------------------------------
# 6) Run the simulation loop
# ------------------------------------------------------------------
time_step = 1e-4
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(time_step)
    app.EndScene()

# EOF