import math as m
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
minertia.SetArea(m.pi * (beam_ro ** 2 - beam_ri ** 2))
minertia.SetIyy((m.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4))
minertia.SetIzz((m.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((m.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4))
melasticity.SetIzz((m.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4))
melasticity.SetJ((m.pi / 2.0) * (beam_ro ** 4 - beam_ri ** 4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)

msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)




builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  
                  msection,  
                  20,  
                  chrono.ChVector3d(0, 0, 0),  
                  chrono.ChVector3d(beam_L, 0, 0),  
                  chrono.VECT_Y,  
                  1)  

node_mid = builder.GetLastBeamNodes()[m.floor(len(builder.GetLastBeamNodes()) / 2.0)]



mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  
                       chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Z))
    
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)


truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1],
                   truss,
                   chrono.ChFramed(builder.GetLastBeamNodes()[-1].GetPos()))
sys.Add(bearing)


rotmotor1 = chrono.ChLinkMotorRotationSpeed()


rotmotor1.Initialize(builder.GetLastBeamNodes()[0],  
                     truss,  
                     chrono.ChFramed(builder.GetLastBeamNodes()[0].GetPos(),
                                     chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Y))
                     
                     )
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)
rotmotor1.SetMotorFunction(f_ramp)



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


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
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