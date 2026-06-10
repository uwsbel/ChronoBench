import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





try:
    Vec = chrono.ChVector3d
except AttributeError:
    Vec = chrono.ChVectorD


def vxyz(v):
    
    return v.x, v.y, v.z





system = chrono.ChSystemSMC()

try:
    system.SetGravitationalAcceleration(Vec(0.0, -9.81, 0.0))
except AttributeError:
    system.Set_G_acc(Vec(0.0, -9.81, 0.0))


try:
    solver = chrono.ChSolverMINRES()
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.SetVerbose(False)
    system.SetSolver(solver)
except Exception:
    system.SetSolverType(chrono.ChSolver.Type_MINRES)

try:
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
except Exception:
    pass





ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(Vec(0.0, 0.0, 0.0))
system.Add(ground)





mesh = fea.ChMesh()
system.Add(mesh)


try:
    mesh.SetAutomaticGravity(True)
except Exception:
    pass


beam_length = 1.0
num_elements = 20
dx = beam_length / num_elements



section = fea.ChBeamSectionCable()
section.SetDiameter(0.020)          
section.SetDensity(1000.0)          
section.SetYoungModulus(2.0e7)      
section.SetRayleighDamping(0.02)



nodes = []
initial_slope = Vec(1.0, 0.0, 0.0)

for i in range(num_elements + 1):
    x = i * dx
    node = fea.ChNodeFEAxyzD(Vec(x, 0.0, 0.0), initial_slope)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elements):
    elem = fea.ChElementCableANCF()
    elem.SetNodes(nodes[i], nodes[i + 1])
    elem.SetSection(section)
    mesh.AddElement(elem)







hinge = fea.ChLinkNodeFrame()
hinge.Initialize(nodes[0], ground)
system.Add(hinge)









vis_beam = fea.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-0.5, 0.5)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)


vis_nodes = fea.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_nodes.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.012)

try:
    mesh.AddVisualShapeFEA(vis_beam)
    mesh.AddVisualShapeFEA(vis_nodes)
except AttributeError:
    mesh.AddVisualShape(vis_beam)
    mesh.AddVisualShape(vis_nodes)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono ANCF Cable Beam - Hinged End Under Gravity")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(Vec(0.55, 0.25, 1.25), Vec(0.50, -0.20, 0.0))
vis.AddTypicalLights()


realtime_timer = chrono.ChRealtimeStepTimer()





time_step = 5.0e-4
frame = 0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    
    if frame % 100 == 0:
        t = system.GetChTime()

        p_hinge = nodes[0].GetPos()
        p_mid = nodes[num_elements // 2].GetPos()
        p_tip = nodes[-1].GetPos()

        hx, hy, hz = vxyz(p_hinge)
        mx, my, mz = vxyz(p_mid)
        tx, ty, tz = vxyz(p_tip)

        print(
            f"t = {t:7.4f} s | "
            f"hinge = ({hx:+.3f}, {hy:+.3f}, {hz:+.3f}) | "
            f"mid = ({mx:+.3f}, {my:+.3f}, {mz:+.3f}) | "
            f"tip = ({tx:+.3f}, {ty:+.3f}, {tz:+.3f})"
        )

    frame += 1
    realtime_timer.Spin(time_step)