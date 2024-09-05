import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()


beam_material = fea.ChMaterialBeamIGA()
beam_section = fea.ChBeamSectionCosserat(beam_material)
beam_element = fea.ChElementBeamIGA()
beam_element.SetBeamSection(beam_section)
beam_mesh = fea.ChMesh()

beam = fea.ChModelBeamIGA()
beam.AddElement(beam_element)
beam.AddMesh(beam_mesh)
my_system.Add(beam)


flywheel = chrono.ChBodyEasyCylinder(1, 0.5, 1000, True, True)
flywheel.SetPos(chrono.ChVectorD(0, 0, 0))  
my_system.Add(flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, -1)), flywheel, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1)))
motor.SetSpeedFunction(chrono.ChFunction_Const(5))  
my_system.Add(motor)


visualization = fea.ChVisualizationFEAmesh(beam)
visualization.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
visualization.SetColorscaleMinMax(0, 5)
visualization.SetSmoothFaces(True)
visualization.SetWireframe(False)


my_application = irr.ChIrrApp(my_system, 'Jeffcott Rotor Simulation', irr.dimension2du(1024, 768))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalCamera(irr.vector3df(0, 3, -6))
my_application.AddTypicalLights()
my_application.AddLightWithShadow(irr.vector3df(1.5, 5.5, -2.5), irr.vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512, irr.SColorf(0.8, 0.8, 1))
my_application.AddLightWithShadow(irr.vector3df(-1.5, 5.5, -2.5), irr.vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512, irr.SColorf(1, 0.8, 0.5))
my_application.SetTimestep(0.01)


my_application.AssetBindAll()
my_application.AssetUpdateAll()
my_application.Run()