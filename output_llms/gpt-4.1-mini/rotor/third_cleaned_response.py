import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr



class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1=40, A2=10, T1=0.03, T2=0.07, T3=0.15, w=20):
        super().__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def Get_y_dx(self, x):
        
        
        

        

        return self.GetVal(x)

    def GetVal(self, x):
        
        if x < 0:
            return 0.0
        if 0 <= x < self.T1:
            return self.A1 * x / self.T1
        elif self.T1 <= x < self.T2:
            return self.A1
        elif self.T2 <= x < self.T3:
            
            return self.A1 - (self.A1 - self.A2) * (x - self.T2) / (self.T3 - self.T2)
        else:
            return self.A2 * m.sin(self.w * (x - self.T3))


sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)




mesh.SetAutomaticGravity(True)  

sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = m.pi  




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
msection.SetDrawCircularRadius(beam_ro)  



builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  
                  msection,  
                  20,  
                  chrono.ChVectorD(0, 0, 0),  
                  chrono.ChVectorD(beam_L, 0, 0),  
                  chrono.VECT_Y,  
                  1)  


last_nodes = builder.GetLastBeamNodes()
node_mid = last_nodes[int(len(last_nodes)/2)]



mbodyflywheel = chrono.ChBodyEasyCylinder(0.24, 0.1, 7800)  


mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))  


mbodyflywheel.SetRot(chrono.ChQuaternionD().Set_AngAxis(CH_PI / 2.0, chrono.VECT_Z))

sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)


truss = chrono.ChBody()
truss.SetBodyFixed(True)  
sys.Add(truss)


bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.SetName("bearing_end")
bearing.Initialize(last_nodes[-1],  
                   truss,
                   chrono.ChFrameD(last_nodes[-1].GetPos())  
                   )
sys.Add(bearing)


rotmotor1 = chrono.ChLinkMotorRotationSpeed()


rotmotor1.Initialize(last_nodes[0],  
                     truss,  
                     chrono.ChFrameD(last_nodes[0].GetPos(),
                                     chrono.ChQuaternionD().Set_AngAxis(CH_PI / 2.0, chrono.VECT_Y))
                     
                     )
sys.Add(rotmotor1)


f_myfun = ChFunctionMyFun()
rotmotor1.SetMotorFunction(f_myfun)



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
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)






sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)