import math as m
import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)                                                          # add mesh early (truth order)

mesh.SetAutomaticGravity(True, 2)                                      # cubic IGA needs >=2 integration pts
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up, gravity down

beam_L = 6                                                             # shaft length [m]
beam_ro = 0.050                                                        # outer radius [m]
beam_ri = 0.045                                                        # inner radius [m] (hollow)
CH_PI = 3.1456

# IGA Cosserat section — hollow circular tube
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)                                              # steel density [kg/m3]
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)                                     # steel Young's modulus
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)                                # visual radius (no Ixx overwrite)

# Build IGA beam with linear order (order=1)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,          # mesh to fill
                  msection,      # section properties
                  20,            # number of spans
                  chrono.ChVector3d(0, 0, 0),        # start A
                  chrono.ChVector3d(beam_L, 0, 0),   # end B
                  chrono.VECT_Y,                      # section Y direction
                  1)                                  # order 1 = linear IGA

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]

# Flywheel — cylinder body at mid-span
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # R, h, density
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),            # offset up slightly
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)          # rotate so cylinder axis aligns X
    )
)
sys.Add(mbodyflywheel)

# Weld flywheel to mid-beam node
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Fixed truss (ground)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# End bearing at the BACK node — constrain translations + ry,rz; free rx (spin)
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes().back(),
    truss,
    chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos())
)
sys.Add(bearing)

# Rotational motor at FRONT node — drives beam spin
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes().front(),                                # slave (beam end)
    truss,                                                             # master (ground)
    chrono.ChFramed(
        builder.GetLastBeamNodes().front().GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)         # motor axis along X
    )
)
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(40, 0.2)                               # sine: amplitude=40 rad/s, freq=0.2 Hz
rotmotor1.SetMotorFunction(f_ramp)

# FEA visualization — add to mesh before vis.Initialize()
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)                       # surface FEM visualization
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)                       # node coordinate-system glyphs
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht window — Initialize FIRST, add scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Pardiso MKL solver (after vis setup, matching truth order)
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# HHT timestepper
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

sys.DoStaticLinear()                                                   # pre-solve under gravity

# simulation parameters
time_step = 0.002                                                      # 2 ms — IGA rotor
sim_end   = 10.0                                                       # 10 s
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # untagged cadence constant


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
