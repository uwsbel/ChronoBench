import math as m
import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC for stiff FEA
mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)                                     # 2 integration points for cubic IGA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))     # lower gravity (Mars-like)

beam_L = 10                                                           # beam length (changed from 6)
beam_ro = 0.060                                                       # outer radius (changed from 0.050)
beam_ri = 0.055                                                       # inner radius (changed from 0.045)

CH_PI = 3.1456                                                        # pi approximation used by truth

# Cross-section inertia properties for hollow circular section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)                                             # steel density kg/m^3
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))        # annular area
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))  # area moment of inertia Iyy
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))  # area moment of inertia Izz

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)                                    # steel Young's modulus Pa
melasticity.SetShearModulusFromPoisson(0.3)                           # shear modulus from Poisson ratio
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))  # polar moment of inertia J

msection = fea.ChBeamSectionCosserat(minertia, melasticity)           # combine inertia + elasticity
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)                               # draw radius (not overwriting Iyy/J)

# Build the IGA beam with 20 spans along X axis
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,                                               # mesh to add elements to
                  msection,                                           # cross-section properties
                  20,                                                 # number of spans
                  chrono.ChVector3d(0, 0, 0),                        # start point
                  chrono.ChVector3d(beam_L, 0, 0),                   # end point
                  chrono.VECT_Y,                                      # suggested Y direction
                  1)                                                  # order 1 (linear)

# Get the midpoint node for flywheel attachment
node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]

# Create flywheel body and attach at beam midpoint
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # radius=0.30 (changed from 0.24)
mbodyflywheel.SetCoordsys(chrono.ChCoordsysd(
    node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),               # center with Y offset
    chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)))            # rotate so cylinder axis on X
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()                                      # rigid weld node to flywheel
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Fixed truss (ground reference for bearings and motor)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# End bearing — constrains translation in Y,Z and rotation in Y,Z; free in X, rot X
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(), truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Rotational motor at the front node (drives beam spin)
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes().front(),                               # slave (beam front node)
    truss,                                                            # master (ground)
    chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                    chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)))  # motor axis along X
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)                              # Sine(amp=60, freq=0.1) changed from Sine(40,0.2)
rotmotor1.SetMotorFunction(f_ramp)

# FEA visualization — surface deformation coloring
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)                      # mesh as ctor arg (9.0.0)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # plain deformed surface
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

# FEA visualization — node coordinate system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # triad glyphs
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht visualization — Initialize FIRST then add scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()                                                      # MUST be before AddCamera/SkyBox
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # camera at (0,2,8) changed from (0,1,4)
vis.AddTypicalLights()

# Pardiso MKL direct solver for stiff beam stiffness matrices
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)
sys.DoStaticLinear()                                                  # pre-solve static equilibrium

time_step = 0.002                                                     # timestep for IGA rotor (truth uses 0.002)
sim_end = 20.0                                                        # simulation duration
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # cadence: frames per render


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
