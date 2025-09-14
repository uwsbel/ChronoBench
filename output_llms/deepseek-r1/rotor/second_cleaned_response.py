import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  

beam_L = 10  
beam_ro = 0.060  
beam_ri = 0.055  

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(chrono.CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((chrono.CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, 
                  chrono.ChVector3d(0, 0, 0), 
                  chrono.ChVector3d(beam_L, 0, 0), 
                  chrono.VECT_Y, 3)  

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]


mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)
mbodyflywheel.SetCoordsys(chrono.ChCoordsysd(
    node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
    chrono.QuatFromAngleAxis(chrono.CH_PI / 2.0, chrono.VECT_Z)
))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


bearing = fea.ChLinkNodeSlaveFrame()
bearing.Initialize(builder.GetLastBeamNodes().back(), 
                   truss, 
                   chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)


helper_motor = chrono.ChBody()
helper_motor.SetPos(builder.GetLastBeamNodes().front().GetPos())
sys.Add(helper_motor)

fixation = fea.ChLinkNodeSlaveFrame()
fixation.Initialize(builder.GetLastBeamNodes().front(), helper_motor, chrono.ChFrameD())
sys.Add(fixation)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(helper_motor,
                     truss,
                     chrono.ChFrameD(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(chrono.CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)  
rotmotor1.SetMotorFunction(f_ramp)

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mesh.AddVisualShapeFEA(mvisualizebeamC)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)