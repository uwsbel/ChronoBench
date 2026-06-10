"""Jeffcott rotor passing through resonance — flexible IGA Cosserat shaft + flywheel.

System type: ChSystemSMC (FEA stiff beam family). A hollow circular elastic shaft
is modeled with IGA (Cosserat) beam elements; a rigid flywheel is rigidly welded to
the mid-span node. One shaft end is driven by a rotary speed motor with a sinusoidal
speed function, while the far end runs in a radial bearing. As the spin speed sweeps,
the shaft whirls and the flexible beam visibly deflects (Jeffcott resonance).

Bodies/links: rigid flywheel (ChBodyEasyCylinder) welded (ChLinkMateFix) to the
mid node; a fixed truss; a radial bearing (ChLinkMateGeneric) at the far node; and a
rotary speed motor (ChLinkMotorRotationSpeed) between the near node and the truss.
Expected behavior: the shaft spins up sinusoidally and bends under low gravity.
"""

import os
import math as m
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics of the flexible rotor (named, no bare literals)
CH_PI = 3.1456
beam_L = 10                      # shaft length (units)
beam_ro = 0.060                  # outer radius of the hollow circular section
beam_ri = 0.055                  # inner radius of the hollow circular section
flywheel_R = 0.30                # flywheel radius
flywheel_h = 0.1                 # flywheel thickness
density = 7800                   # steel density (shaft + flywheel)
n_spans = 20                     # IGA spans along the shaft
iga_order = 1                    # IGA element order (1 = linear)
time_step = 0.002                # IGA rotor timestep
sim_end = 6.0                    # bounded recording duration
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))            # precomputed once

# Section area / second moments for a hollow circular section (precomputed once).
sec_area = CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2))
sec_I = (CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4))
sec_J = (CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4))


# === System & gravity === SMC system (FEA family); reduced-gravity environment
sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(True, 2)   # >=2 integration points/element for FE gravity precision
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))   # low-gravity environment

# === FEA section === hollow circular Cosserat section shared by all shaft elements
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(density)
minertia.SetArea(sec_area)
minertia.SetIyy(sec_I)
minertia.SetIzz(sec_I)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(sec_I)
melasticity.SetIzz(sec_I)
melasticity.SetJ(sec_J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)   # SetAsCircularSection would overwrite Iyy/Izz/J

# FEA beam: no contact material needed — driven by constraints + gravity + motor only.

# === Beam === IGA Cosserat shaft from origin to (beam_L,0,0)
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, n_spans,
                  chrono.ChVector3d(0, 0, 0),
                  chrono.ChVector3d(beam_L, 0, 0),
                  chrono.VECT_Y,
                  iga_order)

beam_nodes = builder.GetLastBeamNodes()                # cache: container kept (SWIG GC guard)
node_front = beam_nodes.front()
node_back = beam_nodes.back()
node_mid = beam_nodes[m.floor(beam_nodes.size() / 2.0)]

# === Bodies === flywheel welded to the mid node of the shaft
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, flywheel_R, flywheel_h, density)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === Joints / constraints === far-end radial bearing + driving speed motor at near end
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(node_back, truss, chrono.ChFramed(node_back.GetPos()))
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(node_front, truss,
                     chrono.ChFramed(node_front.GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)   # sinusoidal motor speed (ampl, freq)
rotmotor1.SetMotorFunction(f_ramp)

# === FEA visualization === deformed surface field + node-CSYS glyphs
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
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # gravity is along -Y
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# === Solver === MKL Pardiso direct solver (stiff IGA beam stiffness matrix)
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Pre-solve a static linear step to settle the shaft under load before the dynamics.
sys.DoStaticLinear()

# === Main loop === render-cadence outer loop, batch physics steps between frames

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
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
