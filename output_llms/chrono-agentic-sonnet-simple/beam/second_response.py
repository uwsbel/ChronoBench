import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()  # FEA requires SMC

# MKL Pardiso solver — required for stiff Euler beams
sys.SetSolver(mkl.ChSolverPardisoMKL())

# HHT timestepper, canonical-minimal form (truth uses exactly these two calls)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# ---- Beam section properties ----
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsCircularSection(0.012)        # 12mm diameter circular section
msection.SetDensity(7800)                   # steel density kg/m^3
msection.SetYoungModulus(0.02e9)           # reduced modulus to allow visible deformation
msection.SetShearModulusFromPoisson(0.3)   # derive shear G from Poisson ratio
msection.SetRayleighDamping(0.001)

# FEA mesh (all beams share one mesh)
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)             # enable automatic gravity

# ---- First beam: manually created nodes and elements ----
# Nodes along X from x=0 to x=0.5
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))    # root node
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0)))  # mid node
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0)))  # tip node

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Elements for the first beam (manual construction)
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Fix hnode1 using ChLinkMateGeneric (comment out: hnode1.SetFixed(True))
truss = chrono.ChBody()       # fixed ground body for constraints
truss.SetFixed(True)
sys.Add(truss)

constr1 = chrono.ChLinkMateGeneric()   # constraint fixing all 6 DOF of hnode1 to ground
constr1.Initialize(hnode1, truss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr1)
constr1.SetConstrainedCoords(True, True, True,   # tx, ty, tz
                              True, True, True)   # rx, ry, rz

# ---- Euler-Bernoulli beam setup using ChBuilderBeamEuler ----
# Build beam from A=(0, 0, -0.1) to B=(0.2, 0, -0.1), Y up, 5 elements
builder = fea.ChBuilderBeamEuler()
builder.BuildBeam(
    mesh, msection, 5,
    chrono.ChVector3d(0, 0, -0.1),    # point A
    chrono.ChVector3d(0.2, 0, -0.1),  # point B
    chrono.ChVector3d(0, 1, 0),       # Y up direction
)

# Keep strong reference to builder nodes (SWIG GC pitfall: never index temporary)
beam_nodes = builder.GetLastBeamNodes()

# Fix the last node of the builder beam
beam_nodes.back().SetFixed(True)

# Apply force to the first node of the builder beam section
beam_nodes.front().SetForce(chrono.ChVector3d(0, -1, 0))  # downward force at free end

sys.Add(mesh)

# FEA visualization — surface: bending moment Mz field
vis_surface = chrono.ChVisualShapeFEA(mesh)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Mz bending moment
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

# FEA visualization — glyphs: node coordinate systems
vis_glyph = chrono.ChVisualShapeFEA(mesh)
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# Irrlicht visualization — Initialize() FIRST, then scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA scenes
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Euler-Bernoulli Beam with ChBuilderBeamEuler")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.4), chrono.ChVector3d(0.1, 0.0, -0.05))
vis.AddTypicalLights()

time_step = 1e-3   # 1ms timestep for stiff Euler beams
sim_end = 5.0      # 5 seconds
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
