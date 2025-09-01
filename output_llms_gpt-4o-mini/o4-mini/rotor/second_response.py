import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# -------------------------------------------------------------------------
# 1) System and mesh setup
# -------------------------------------------------------------------------
sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

# Tell the mesh to pick up system gravity automatically
mesh.SetAutomaticGravity(True, 2)
# 4) Lower gravity environment
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))

# -------------------------------------------------------------------------
# 2) Beam geometry & properties
# -------------------------------------------------------------------------
beam_L  = 10.0               # 1) changed from 6 to 10
beam_ro = 0.060              # 2) changed from 0.050
beam_ri = 0.055              # 2) changed from 0.045

# Use high-precision Pi
CH_PI = math.pi

# Cross-section inertia and elasticity for Cosserat beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800.0)
minertia.SetArea(  CH_PI * (beam_ro**2 - beam_ri**2) )
minertia.SetIyy( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )
minertia.SetIzz( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )
melasticity.SetIzz( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )
melasticity.SetJ(   (CH_PI/2.0) * (beam_ro**4 - beam_ri**4) )

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# -------------------------------------------------------------------------
# 3) Build the IGA beam
# -------------------------------------------------------------------------
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,      # mesh container
                  msection,  # beam section
                  20,        # number of spans
                  chrono.ChVector3d(0, 0, 0),             # start
                  chrono.ChVector3d(beam_L, 0, 0),        # end
                  chrono.VECT_Y,                           # up direction
                  1)                                       # order (1=linear)

# Get the list of beam nodes
nodes = builder.GetLastBeamNodes()
n_nodes = nodes.size()

# pick the middle node
mid_idx  = int(n_nodes/2)
node_mid = nodes[mid_idx]

# -------------------------------------------------------------------------
# 4) Create and attach the flywheel
# -------------------------------------------------------------------------
# 3) radius changed to 0.30
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800.0)
# move it to the beam midpoint + small offset
mbodyflywheel.SetFrame_REF_to_abs(
    chrono.ChFrameD(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)

# fix the cylinder to the beam node
fix = chrono.ChLinkMateFix()
fix.Initialize(node_mid, mbodyflywheel)
sys.Add(fix)

# -------------------------------------------------------------------------
# 5) Create the truss support and bearings at the ends
# -------------------------------------------------------------------------
truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

# Left bearing (at node 0)
bearingL = chrono.ChLinkMateGeneric(False, True, True,  False, True, True)
bearingL.Initialize(
    nodes[0], 
    truss,
    chrono.ChFrameD(nodes[0].GetPos())
)
sys.Add(bearingL)

# Right bearing (at last node)
bearingR = chrono.ChLinkMateGeneric(False, True, True,  False, True, True)
bearingR.Initialize(
    nodes[n_nodes-1],
    truss,
    chrono.ChFrameD(nodes[n_nodes-1].GetPos())
)
sys.Add(bearingR)

# -------------------------------------------------------------------------
# 6) Add the motor at the left end
# -------------------------------------------------------------------------
rotmotor = chrono.ChLinkMotorRotationSpeed()
rotmotor.Initialize(
    nodes[0],    # slave
    truss,       # master
    chrono.ChFrameD(
        nodes[0].GetPos(),
        chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Y)
    )
)
sys.Add(rotmotor)

# 5) Change motor function to Sine(60, 0.1)
f_ramp = chrono.ChFunctionSine(60.0, 0.1)
rotmotor.SetMotorFunction(f_ramp)

# -------------------------------------------------------------------------
# 7) Visualize the FEA beam
# -------------------------------------------------------------------------
vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)

# -------------------------------------------------------------------------
# 8) Irrlicht setup
# -------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('IGA Beam Jeffcott Rotor')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 2, 8),            # 6) camera moved from (0,1,4)
    chrono.ChVector3d(beam_L/2.0, 0, 0)
)
vis.AddTypicalLights()

# -------------------------------------------------------------------------
# 9) Solver and run
# -------------------------------------------------------------------------
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# perform a static solve to settle initial constraints
sys.DoStaticLinear()

# then run dynamic
step_size = 0.002
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(step_size)