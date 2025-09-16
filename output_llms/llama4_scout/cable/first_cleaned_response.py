import numpy as np
from chrono import chrono_types
from chrono import ChronoEngine
from chrono import Irrlicht
from chrono import ChBeam


chrono_types.InitChrono()


system = chrono_types.ChSystem()


app = Irrlicht.ChIrrApp(system, "ANCF Beam Simulation")


system.Set_G_acc(chrono_types.ChVectorD(0, -9.81, 0))


ground = chrono_types.ChBody()
ground.SetFixed(True)
system.AddBody(ground)


beam = ChBeam.ChBeam()
beam.SetBeamLength(10.0)  
beam.SetNumElements(10)   
beam.SetNumSections(1)    
beam.SetSectionRadius(0.1) 
beam.SetYoungModulus(2e6)  
beam.SetDensity(7850)      


beam.Initialize(ground, chrono_types.ChVectorD(0, 0, 0), chrono_types.ChQuaternionD(1, 0, 0, 0))


system.Add(beam)


viz = Irrlicht.ChIrrVisualization(beam)
app.Add(viz)


system.Set_Timestep(0.01)
system.Set_StopTime(10.0)


app.Run()


chrono_types.ChronoEngine.Finalize()