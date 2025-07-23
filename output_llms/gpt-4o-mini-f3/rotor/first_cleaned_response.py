import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                         2)  
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0));

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1456




minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800);
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)));
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)));

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

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

node_mid = fea.CastToChNodeFEACoordinateSystem(builder.GetLastBeamNodes()[1])
node_end = fea.CastToChNodeFEACoordinateSystem(builder.GetLastBeamNodes()[2])



mflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  
mflywheel.SetCoordsRelativeToBody(chrono.ChCoordsysd(
    node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  
    chrono.QuatFromAngleLine(CH_PI / 2.0, chrono.VECT_Z)))  

sys.Add(mflywheel)

myjoint = chrono.ChLinkMateGeneric()
myjoint.Initialize(node_mid,  
                   mflywheel,  
                   False,  
                   node_mid.GetPos(),  
                   node_mid.GetPos() + chrono.ChVector3d(0, 0, 1))  
sys.Add(myjoint)




motor_funct = chrono.ChFunctionConst(100.0m)  

mymotor = chrono.ChLinkMotorRotationSpeed()
mymotor.Initialize(builder.GetLastBeamNodes()[0],  
                   builder.GetLastBeamNodes()[1],  
                   chrono.ChAxis(arr=chrono.ChVector3d(0, 0, 1)))  
mymotor.SetMotorFunction(motor_funct)
sys.Add(mymotor)


for ia in range(0, builder.GetLastBeamNodeIndex()):
    builder.GetLastBeamNodes()[ia].SetIconMode(fea.ChNodeFEAbase.IconMode_DM_TRIANGULAR_MESH)
    builder.GetLastBeamNodes()[ia].SetSymbolscale(0.01)

for elem in mesh.GetElements():
    elem.SetSymbolsThickness(0.006)
    elem.SetSymbolsThickness(0.006)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msystem.SetVerbose(False)
msystem.DoTimeline()