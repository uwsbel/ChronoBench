import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Test: IGA beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))
vis.AddTypicalLights()





mesh = fea.ChMesh()
sys.Add(mesh)


beam_L = 3  
beam_rad = 0.04  


builder = fea.ChBuilderBeamIGA()
section_cyl = fea.ChBeamSectionEulerAdvanced()
section_cyl.SetDensity(1600)  
section_cyl.SetYoungModulus(3.5e9)  
section_cyl.SetShearModulus(3.5e9 / (2.0 * (1.0 + 0.3)))  
section_cyl.SetRadii(beam_rad, beam_rad + 0.01)  
section_cyl.SetRayleighDamping(0.000)  


builder.BuildBeam(mesh, section_cyl,  
                  2,  
                  chrono.ChVector3d(0, 0, 0),  
                  chrono.ChVector3d(beam_L, 0, 0),  
                  chrono.VECT_Y,  
                  1)  


node_div_6 = builder.GetLastBeamNodes()[1]  
node_div_6.SetFixed(True)  


flywheel = chrono.ChBody()
flywheel.SetMass(0.96)  
flywheel.SetInertiaXX(chrono.ChVector3d(0.032, 0.96, 0.96))  
flywheel.SetFixed(False)  
flywheel.SetFrictIon(0)  
flywheel.SetRestitution(0)  
flywheel.SetContactMethod(chrono.ChContactMethod_NSC)  

flywheel.AddAsset(chrono.ChVisualShapeCylinder(.05, .2))


sys.Add(flywheel)


spjoint = chrono.ChLinkLockSpherical()

spjoint.Initialize(flywheel,
                  node_div_6,
                  chrono.ChVector3d(beam_L / 2, 0, 0))

sys.AddLink(spjoint)


builder2 = fea.ChBuilderBeamIGA()
section_box = fea.ChBeamSectionEulerAdvanced()
section_box.SetDensity(1600)  
section_box.SetYoungModulus(3.5e9)  
section_box.SetRayleighDamping(0.000)  
section_box.SetAsRectangularSection(0.05, 0.15)  


rot90 = chrono.ChQuaterniond()
rot90.SetFromAngleAxis(chrono.CHPI_2, chrono.VECT_X)
mcs = chrono.ChMatrix33d(rot90)


builder2.BuildBeam(mesh, section_box,
                   2,  
                   chrono.ChVector3d(beam_L / 2, 0.1, 0),  
                   chrono.ChVector3d(beam_L / 2, 0.25, 0),  
                   mcs,  
                   1)  


motor = chrono.ChLinkMotorRotationSpeed()

flywheel_motor = chrono.ChBody()
flywheel_motor.SetFixed(False)
flywheel_motor.AddVisualShape(chrono.ChVisualShapeSphere(0.1))
motor.SetMotorFlywheel(flywheel_motor)  
sys.Add(flywheel_motor)  
my_gear = chrono.ChLinkLockScrew()  
my_gear.Initialize(flywheel_motor,
                  sys.GetBodyFromFrame(motor.Frame2),
                  motor.Frame2)
sys.Add(my_gear)  


motor.Initialize(builder.GetLastBeamNodes()[-1],  
                chrono.ChNullBody(),  
                chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))  
sys.AddLink(motor)  

rot_funct = chrono.ChFunctionConst(35)
motor.SetMotorFunction(rot_funct)


visualizebeamA = chrono.ChVisualShapeContainer()
mesh.AddVisualShapeContainer(visualizebeamA)


g_strain = fea.ChVisualFEMdata(mesh)
g_strain.SetFEMdataType(chrono.ChVisualFEMdata.E_PLOT_UNITSTRAIN) 
g_strain.SetThickness(0.010)  
g_strain.SetSmoothFaces(True)  
g_strain.SetSmoothFalloff(True)  
visualizebeamA.AddVisualShape(g_strain)  

g_stress = fea.ChVisualFEMdata(mesh)
g_stress.SetFEMglyphType(chrono.ChVisualFEMdata.E_GLYPH_NODE_STRESS_RADII) 
g_stress.SetFEMdataType(chrono.ChVisualFEMdata.E_PLOT_NONE) 
g_stress.SetSymbolsThickness(0.01)  
g_stress.SetStressTensorScale(0.05)  
g_stress.SetSmoothFaces(True)  
visualizebeamA.AddVisualShape(g_stress)  


mesh.SetAutomaticGravity(False)  
mesh.SetVisualFEMstyle(chrono.ChVisualFEMdata.Style_ELEM_WIREframe)  

coordsys = chrono.ChVisualShapeFrame(0.3)
sys.AddVisualShape(coordsys, chrono.ChFramed(chrono.VECT3_ZERO))


solver = chrono.ChSolverMINRES()
solver.SetDiagonalPreconditioning(True)
solver.SetMaxIterations(100)
solver.SetTolerance(1e-10)
solver.SetVerbose(False)
sys.SetSolver(solver)


ts = chrono.ChTimestepperHHT()
ts.SetAlpha(-0.2)
sys.SetTimestepper(ts)


timestep = 0.001


while vis.Run():
    sys.DoStepDynamics(timestep)  
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()