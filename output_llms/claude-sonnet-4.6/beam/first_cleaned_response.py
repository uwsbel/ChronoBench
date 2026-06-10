import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr




print("Initializing PyChrono Beam FEA Simulation...")


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




mesh = fea.ChMesh()





beam_section = fea.ChBeamSectionEulerAdvanced()


beam_width  = 0.012   
beam_height = 0.025   
beam_section.SetAsRectangularSection(beam_width, beam_height)


beam_section.SetYoungModulus(210e9)   
beam_section.SetGshearModulus(80e9)   
beam_section.SetDensity(7800)         
beam_section.SetBeamRaleyghDamping(0.002)




builder = fea.ChBuilderBeamEuler()


builder.BuildBeam(
    mesh,                                        
    beam_section,                                
    8,                                           
    chrono.ChVectorD(0.0, 0.0, 0.0),            
    chrono.ChVectorD(0.4, 0.0, 0.0),            
    chrono.ChVectorD(0, 1, 0)                    
)


builder.GetLastBeamNodes().front().SetFixed(True)


tip_node_1 = builder.GetLastBeamNodes().back()
tip_node_1.SetForce(chrono.ChVectorD(0, -2.0, 0))   


builder.BuildBeam(
    mesh,
    beam_section,
    6,
    tip_node_1.GetPos(),                         
    tip_node_1.GetPos() + chrono.ChVectorD(0, 0, 0.3),  
    chrono.ChVectorD(0, 1, 0)
)

tip_node_2 = builder.GetLastBeamNodes().back()
tip_node_2.SetForce(chrono.ChVectorD(0, -1.5, 0.5))  


builder.BuildBeam(
    mesh,
    beam_section,
    5,
    chrono.ChVectorD(0.0, 0.0, 0.0),
    chrono.ChVectorD(0.2, 0.2, 0.15),
    chrono.ChVectorD(0, 1, 0)
)
builder.GetLastBeamNodes().front().SetFixed(True)
builder.GetLastBeamNodes().back().SetForce(chrono.ChVectorD(0.5, -1.0, 0.3))




tip_node_2.SetMass(0.05)   




mesh.SetAutomaticGravity(True)


system.Add(mesh)






vis_beam_A = chrono.ChVisualShapeFEA(mesh)
vis_beam_A.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_A.SetColorscaleMinMax(-0.4, 0.4)
vis_beam_A.SetSmoothFaces(True)
vis_beam_A.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_A)


vis_beam_B = chrono.ChVisualShapeFEA(mesh)
vis_beam_B.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_beam_B.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_B.SetSymbolsThickness(0.006)
vis_beam_B.SetSymbolsScale(0.01)
vis_beam_B.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_B)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono - Beam FEA Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.3, 0.3, 0.7), chrono.ChVectorD(0.2, 0.0, 0.15))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVectorD(1.5, 5.5, -2.5),
    chrono.ChVectorD(0.0, 0.0, 0.0),
    10, 1, 11, 55
)





solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableWarmStarting(True)
system.SetSolver(solver)


integrator = chrono.ChTimestepperHHT(system)
integrator.SetAlpha(-0.2)
integrator.SetMaxiters(10)
integrator.SetAbsTolerances(1e-5)
integrator.SetMode(chrono.ChTimestepperHHT.POSITION)
integrator.SetScaling(True)
system.SetTimestepper(integrator)




timestep = 0.001   
frame    = 0
print_interval = 100

print("Starting simulation loop... Close the window to exit.")
print(f"  Timestep : {timestep} s")
print(f"  Nodes    : {mesh.GetNnodes()}")
print(f"  Elements : {mesh.GetNelements()}")

while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    chronoirr.drawAllCsyses(vis, 0.02)

    vis.EndScene()

    system.DoStepDynamics(timestep)
    frame += 1

    
    if frame % print_interval == 0:
        t = system.GetChTime()
        tip_pos = tip_node_1.GetPos()
        print(f"  t={t:.3f}s | Tip-1 pos: "
              f"({tip_pos.x:.4f}, {tip_pos.y:.4f}, {tip_pos.z:.4f})")

print("Simulation finished.")