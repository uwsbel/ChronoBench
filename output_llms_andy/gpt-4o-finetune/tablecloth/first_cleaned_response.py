import pychrono.core as chrono  
import pychrono.irrlicht as chronoirr  
import pychrono.fea as fea  
import pychrono.pardisomkl as pardisomkl  
import math  


sys = chrono.ChSystemNSC()


my_viscoelasticity = 1e7  
my_E = 3.5e9  
my_v = 0.3  


hlength = 1.1  
hhwidth = 0.5  
thickness = 0.005  


mesh = fea.ChMesh()


 msection = fea.ChBeamSectionEulerAdvanced()
 msection.SetDiameter(0.001)
 msection.SetShearCenterTowardsNeutralAxis()
 msection.SetYoungModulus(my_E)
 msection.SetRayleighDamping(0.))  
 msection.SetUseMitcalCSectionShearFormulation(True)


mtruss_load_case = fea.ChLoadTrussAbsolute()
mtruss_load_case.Add(chrono.VECT_X, 1, -1)
mtruss_load_case.Add(chrono.VECT_Y, 1, -1)
mtruss_load_case.Add(chrono.VECT_Z, 1, -1)


def make_straight_beam(p1, p2):
    
    segment = fea.ChBeamSegmentEuler()
    segment.BuildBeam(p1, p2, msection)

    
    segment.Load(mtruss_load_case)

    return segment


last_left = mesh.AddNode(
    fea.ChNodeFEA0(
        chrono.ChVector3d(-hlength, -hhwidth, 0),  
        chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  
    )
)
last_right = mesh.AddNode(
    fea.ChNodeFEA0(
        chrono.ChVector3d(-hlength, +hhwidth, 0),  
        chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  
    )
)


numdiv = 30  
for i in range(numdiv):
    next_left = mesh.AddNode(
        fea.ChNodeFEA0(
            chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, -hhwidth, 0),  
            chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  
        )
    )
    next_right = mesh.AddNode(
        fea.ChNodeFEA0(
            chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, +hhwidth, 0),  
            chrono.ChVector3d(chrono.GetNaN(), chrono.GetNaN(), chrono.GetNaN())  
        )
    )
    
    
    line1 = fea.ChMesh()
    line1.AddNode(last_left)
    line1.AddNode(next_left)
    beam1 = fea.ChElementBeamEuler()
    beam1.SetSection(msection)
    beam1.Resize(2)
    beam1.SetNodes(line1.GetNode(0), line1.GetNode(1))
    line1.AddElement(beam1)
    mtruss_load_case.AddElement(beam1)
    mload = fea.ChLoadBeamElement()
    mload.AddNext(chrono.VECT_Y, -0.25)
    beam1.AddLoad(mload)
    mesh.AddBeam(line1)

    line2 = fea.ChMesh()
    line2.AddNode(last_right)
    line2.AddNode(next_right)
    beam2 = fea.ChElementBeamEuler()
    beam2.SetSection(msection)
    beam2.Resize(2)
    beam2.SetNodes(line2.GetNode(0), line2.GetNode(1))
    line2.AddElement(beam2)
    mtruss_load_case.AddElement(beam2)
    mesh.AddBeam(line2)

    line3 = fea.ChMesh()
    line3.AddNode(last_left)
    line3.AddNode(last_right)
    beam3 = fea.ChElementBeamEuler()
    beam3.SetSection(msection)
    beam3.Resize(2)
    beam3.SetNodes(line3.GetNode(0), line3.GetNode(1))
    line3.AddElement(beam3)
    mtruss_load_case.AddElement(beam3)
    mesh.AddBeam(line3)

    line4 = fea.ChMesh()
    line4.AddNode(next_left)
    line4.AddNode(next_right)
    beam4=fea.ChElementBeamEuler()
    beam4.SetSection(msection)
    beam4.Resize(2)
    beam4.SetNodes(line4.GetNode(0), line4.GetNode(1))
    line4.AddElement(beam4)
    mtruss_load_case.AddElement(beam4)
    mesh.AddBeam(line4)

    segment = make_straight_beam(chrono.ChVector3d(-hlength + i * 2 * hlength / numdiv, 0, 0),
                                chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, 0, 0))  
    mesh.AddElement(segment)

    segment = make_straight_beam(chrono.ChVector3d(-hlength + i * 2 * hlength / numdiv, -hhwidth / 3, 0),
                                chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, -hhwidth / 3, 0))  
    mesh.AddElement(segment)

    segment = make_straight_beam(chrono.ChVector3d(-hlength + i * 2 * hlength / numdiv, hhwidth / 3, 0),
                                chrono.ChVector3d(-hlength + (i + 1) * 2 * hlength / numdiv, hhwidth / 3, 0))  
    mesh.AddElement(segment)

    last_left = next_left
    last_right = next_right


for i in range(10):
    nodeB = mesh.GetNode(numdiv - 1 + i * (numdiv + 1))
    mfix = chrono.ChLinkMateFix()
    mfix.Initialize(nodeB, chrono.ChFramed())
    sys.Add(mfix)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Kirchhoff-Love shell elements FEA')  
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 0.7, -1.5), chrono.ChVector3d(0, 0, 0))  
vis.AddTypicalLights()  


solver = pardisomkl.ChSolverPardisoMKL()
sys.SetSolver(solver)


solver.SetAutomaticScaleFactor(True)  
sys.SetTimestepperType(chrono.ChTimestepper.Type.HHT)  
solver.GetMaxIterationsGeneral().Set(50)  


hht = chrono.ChTimestepperHHT()
sys.SetTimestepper(hht)
hht.SetAlpha(-0.1)  
hht.SetMaxIterations(100)  
hht.SetAbsTolerances(1e-5, 1e-2)  
hht.SetMode(chrono.ChTimestepper.Mode.NORMAL)  


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)