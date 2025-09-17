import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA_FOLDER')
my_system = chrono.ChSystemNSC()


beam_mesh = fea.ChBeamMesh()
beam_mesh.LoadFromFile('PATH_TO_BEAM_MESH_FILE')
beam_section = fea.ChBeamSectionAdvancedGeneric()
beam_section.SetYoungModulus(2.1e11)
beam_section.SetGwithPoissonRatio(0.3)
beam_section.SetDensity(7850)
beam_section.SetAsRectangularSection(0.01, 0.02)
beam_material = fea.ChBeamMaterial(beam_section)
beam_element = fea.ChElementBeamIGA()
beam_element.SetBeamMesh(beam_mesh)
beam_element.SetBeamMaterial(beam_material)
beam_element.SetMassPerLength(1)
beam_element.SetAlphaDamp(0.005)
beam_element.SetGravity(chrono.ChVectorD(0, -9.81, 0))
my_system.Add(beam_element)


flywheel = chrono.ChBodyEasyCylinder(0.05, 0.05, 0.02, 1000, True, True)
flywheel.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_element.GetNodeN(0), flywheel, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(10))
my_system.Add(motor)


beam_element.AddVisualizationFEAmesh().SetFEMdataType(fea.ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM)
beam_element.AddVisualizationFEAmesh().SetColorscaleMinMax(0, 10)
beam_element.AddVisualizationFEAmesh().SetSmoothFaces(True)

my_application = irr.ChIrrApp(my_system, 'Jeffcott rotor simulation', irr.dimension2du(1024, 768))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalCamera(irr.vector3df(0.0, 0.2, -1.0))
my_application.AddTypicalLights()

my_application.AssetBindAll()
my_application.AssetUpdateAll()


my_application.SetTimestep(0.01)
while(my_application.GetDevice().run()):
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()