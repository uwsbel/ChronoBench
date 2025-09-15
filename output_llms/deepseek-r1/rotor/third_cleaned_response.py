import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
        self.A1 = 40.0   
        self.A2 = 0.2    
        self.T1 = 0.5    
        self.T2 = 1.0    
        self.T3 = 1.5    
        self.w = 40.0    
    
    def GetVal(self, x):
        
        if x < self.T1:
            return 0.0  
        elif x < self.T2:
            
            return self.A1 * (x - self.T1) / (self.T2 - self.T1)
        elif x < self.T3:
            return self.A1  
        else:
            
            return self.A1 + self.A2 * math.sin(self.w * (x - self.T3))
    
    def Clone(self):
        
        new_func = ChFunctionMyFun()
        new_func.A1 = self.A1
        new_func.A2 = self.A2
        new_func.T1 = self.T1
        new_func.T2 = self.T2
        new_func.T3 = self.T3
        new_func.w = self.w
        return new_func


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)


mesh.SetAutomaticGravity(True, 2)  
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


beam_L = 6
beam_ro = 0.050
beam_ri = 0.045


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(math.pi * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((math.pi / 4) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((math.pi / 4) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(minertia.GetIyy())
melasticity.SetIzz(minertia.GetIzz())
melasticity.SetJ(math.pi/2 * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)


builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,                   
    msection,               
    20,                     
    chrono.ChVector3d(0, 0, 0),  
    chrono.ChVector3d(beam_L, 0, 0),  
    chrono.ChVector3d(0, 1, 0),  
    3                       
)


nodes = builder.GetLastBeamNodes()
node_mid = nodes[len(nodes) // 2]


mbodyflywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y, 0.24, 0.1, 7800  
)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  
        chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(0, 0, 1))  
    )
)
sys.Add(mbodyflywheel)


myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)


truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes().back(),  
    truss,                              
    chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())  
)
sys.Add(bearing)


rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes().front(),  
    truss,                              
    chrono.ChFrameD(  
        builder.GetLastBeamNodes().front().GetPos(),
        chrono.QUNIT  
    )
)
sys.Add(rotmotor1)


rotmotor1.SetMotorFunction(ChFunctionMyFun())


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
vis.SetWindowTitle('Modified FEA: Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


sys.DoStaticLinear()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)