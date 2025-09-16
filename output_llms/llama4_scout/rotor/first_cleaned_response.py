import chrono as ch
import chrono.irrlicht as irr
import chrono.fem as fem
import math


ch.CH_init()


system = ch.ChSystem()


beam = fem.ChBeamIGA()
beam.SetNodesSpacing(0.1)  
beam.SetLength(1.0)      
beam.SetSection(fem.ChBeamSectionIGA(0.05, 0.01, 0.01))  
beam.SetMaterial(fem.ChFEMMaterial(200e9, 0.3, 7850))  
system.Add(beam)


flywheel = ch.ChBodyEasyBody(1.0,  
                              0.5,  
                              ch.ChFrame(ch.Vector(0, 0, 0.5), ch.Quaternion(1, 0, 0, 0)))
flywheel.SetPos(ch.Vector(0.5, 0, 0.5))  
system.Add(flywheel)


ch.ChLinkMateFix().Connect(beam, flywheel)


motor = ch.ChLinkMotorRotation()
motor.Init(beam, 
           ch.Frame(ch.Vector(0, 0, 0), ch.Quaternion(1, 0, 0, 0)), 
           ch.Frame(ch.Vector(0, 0, 0), ch.Quaternion(1, 0, 0, 0)))
motor.SetMotorFunction(ch.ChFunction_Sine(0, 1, 0))  
system.Add(motor)


app = irr.ChIrrApp(system, 'Jeffcott Rotor Simulation', 800, 600)


fem_vis = ch.ChFEMVisualization(beam)
fem_vis.SetFEMdataResolution(4)
app.AddVisualization(fem_vis)


app.SetTimestep(0.01)
app.SetCollision(True)


app.Run()