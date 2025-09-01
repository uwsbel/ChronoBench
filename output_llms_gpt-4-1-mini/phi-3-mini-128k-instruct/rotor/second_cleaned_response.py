import math as m
import chrono as chrono
import chrono.fea as fea
import chrono.pardisomkl as mkl
import chrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                         2)  
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0));

beam_L = 10
beam_ro = 0.060
beam_ri = 0.055
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

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]



mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
    
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)


truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)


rotmotor1 = chrono.ChLinkMotorRotationSpeed()


rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  
                     truss,  
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
                     
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
mvisualizebeamC.SetSymbols00)
)






)


11)


)
)

)

)
)
0)
)
)
0,











0)
















)
)




















,
,
























)
)
)
,
,




,



)












,
)












)
)
)



)
)
)
)
)
)









)
)



)
)





)
)
)
)
)
)
.
)
)
)
)
)