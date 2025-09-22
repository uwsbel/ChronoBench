import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        super(ChFunctionMyFun, self).__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w  = w

    
    def Get_y(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif x < self.T2:
            return self.A1
        elif x < self.T3:
            return self.A2
        else:
            return 0.0

    
    def GetVal(self, x):
        return self.Get_y(x)




sys = chrono.ChSystemSMC()

sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)




beam_L  = 6.0
beam_ro = 0.050
beam_ri = 0.045
CH_PI   = m.pi


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800.0)
area = CH_PI * (beam_ro**2 - beam_ri**2)
minertia.SetArea(area)
I   = (CH_PI/4.0)*(beam_ro**4 - beam_ri**4)
minertia.SetIyy(I)
minertia.SetIzz(I)


melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(I)
melasticity.SetIzz(I)
melasticity.SetJ((CH_PI/2.0)*(beam_ro**4 - beam_ri**4))

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

nodes = builder.GetLastBeamNodes()

node_front = nodes[0]
node_mid   = nodes[int(len(nodes)/2)]
node_back  = nodes[-1]





mbodyflywheel = chrono.ChBodyEasyCylinder(0.24, 0.1, 7800.0, True, False)

mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_from_AngAxis(CH_PI/2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)


joint_fly = chrono.ChLinkMateFix()
joint_fly.Initialize(node_mid, mbodyflywheel)
sys.Add(joint_fly)




truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)


bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
frame_back = chrono.ChFrameD(node_back.GetPos())
bearing.Initialize(node_back, truss, frame_back)
sys.Add(bearing)




rotmotor1 = chrono.ChLinkMotorRotationSpeed()
frame_front = chrono.ChFrameD(node_front.GetPos(),
                              chrono.Q_from_AngAxis(CH_PI/2.0, chrono.VECT_Y))
rotmotor1.Initialize(node_front, truss, frame_front)
sys.Add(rotmotor1)


A1, A2 = 20.0, 40.0
T1, T2, T3 = 1.0, 2.0, 5.0
w = 10.0
fun_motor = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(fun_motor)




vis_beamA = chrono.ChVisualShapeFEA(mesh)
vis_beamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beamA)

vis_beamC = chrono.ChVisualShapeFEA(mesh)
vis_beamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beamC.SetSymbolsThickness(0.006)
vis_beamC.SetSymbolsScale(0.01)
vis_beamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beamC)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with Custom Motor Function')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4),
              chrono.ChVector3d(beam_L/2, 0, 0))
vis.AddTypicalLights()




msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)













sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)