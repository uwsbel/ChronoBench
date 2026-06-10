"""FEA Jeffcott rotor passing through resonance (IGA Cosserat beam).

System type: ChSystemSMC (FEA needs a direct solver; Pardiso MKL here).
Model: a slender hollow steel shaft modelled as a single IGA (Cosserat) beam,
spinning about its own axis. A heavy flywheel is rigidly welded to the mid node,
the far end rides in a radial bearing, and a rotational-speed motor at the near
end spins the rotor. The motor speed is governed by a CUSTOM ChFunction subclass
(ChFunctionMyFun) whose GetVal returns a piecewise smooth speed ramp built from
the parameters A1, A2, T1, T2, T3, w — replacing a plain sine drive. As the
imposed speed sweeps upward the rotor whirls through its critical (resonance)
speed and the mid-span deflection grows then settles, the classic Jeffcott
behaviour. World is Y-up with gravity (0, -9.81, 0).

Expected behaviour: the shaft spins up smoothly following the piecewise speed
profile; the welded flywheel whirls about the shaft axis and the beam bows
laterally most strongly while crossing the critical speed.
"""

import math as m
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Parameters === rotor geometry / drive profile (named, no bare literals downstream)
beam_L = 6.0            # shaft length [m]
beam_ro = 0.050        # outer radius [m]
beam_ri = 0.045        # inner radius [m] (hollow shaft)
CH_PI = 3.1456         # pi value used by the section/inertia helpers
time_step = 2e-3       # IGA rotor timestep
sim_end = 2.0          # run long enough to cover the T3=1.25 s speed ramp

# === System & gravity === SMC system, Y-up gravity for this FEA scene
sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)
# at least 2 integration points per cubic IGA element for accurate FE gravity
mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Beam section === Cosserat inertia + elasticity for a hollow circular shaft
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)   # SetAsCircularSection would overwrite Iyy/Izz/J

# === Beam mesh === one straight IGA rod from 0 to beam_L, built along +X
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  msection,
                  20,                                  # number of spans
                  chrono.ChVector3d(0, 0, 0),          # A (start)
                  chrono.ChVector3d(beam_L, 0, 0),     # B (end)
                  chrono.VECT_Y,                       # suggested section Y direction
                  1)                                   # order (1 = linear)

beam_nodes = builder.GetLastBeamNodes()   # cache: keep a strong ref (SWIG GC guard)
node_mid = beam_nodes[m.floor(beam_nodes.size() / 2.0)]
node_first = beam_nodes.front()
node_last = beam_nodes.back()

# === Bodies / constraints === flywheel + truss + bearing + motor
# FEA beam: no contact material needed — driven by constraints + gravity + motor only.

# Flywheel rigidly welded to the mid node (its mass triggers the whirl)
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # R, h, density
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Fixed truss = the stator / ground reference
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Radial bearing at the far end: free along axis (X) + free axial spin, constrains lateral
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(node_last, truss, chrono.ChFramed(node_last.GetPos()))
sys.Add(bearing)

# Speed motor at the near end driving the shaft about its own (X) axis
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(node_first, truss,
                     chrono.ChFramed(node_first.GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)


# === Custom motor function === piecewise smooth speed ramp (replaces the sine drive)
class ChFunctionMyFun(chrono.ChFunction):
    """Imposed motor speed w(x): smooth raised-cosine ramp to A1*w over [0,T1],
    a plateau over [T1,T2], a second raised-cosine ramp to A2*w over [T2,T3],
    then a constant A2*w. Sweeps the rotor through its critical speed."""

    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        A1 = 0.8
        A2 = 1.2
        T1 = 0.5
        T2 = 1.0
        T3 = 1.25
        w = 60
        if x < T1:
            return A1 * w * (1.0 - m.cos(CH_PI * x / T1)) / 2.0
        elif x > T1 and x <= T2:
            return A1 * w
        elif x > T2 and x <= T3:
            return A1 * w + (A2 - A1) * w * (1.0 - m.cos(CH_PI * (x - T2) / (T3 - T2))) / 2.0
        else:
            return A2 * w


f_ramp = ChFunctionMyFun()
rotmotor1.SetMotorFunction(f_ramp)

# === FEA visualization assets === deformed surface field + node coordinate glyphs
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Test FEA: the Jeffcott rotor with IGA beams")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# === Solver / timestepper === Pardiso MKL direct solver for the stiff FE matrices
# Set to a more precise HHT timestepper if needed:
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Pre-solve static step to settle the structure before the dynamic sweep
sys.DoStaticLinear()

# === Main loop === real-time render + step; spin the rotor through resonance
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid FE state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot, drop frame PNGs
