from pychrono import ChSystem, ChBody, ChLink, ChMaterialSurface, ChQuaterniond, ChVectorD
from pychrono.irrlicht import ChIrrApp
import numpy as np


system = ChSystem()


beam_material = ChMaterialSurface()
beam_material.SetDamping(0.1)
beam_material.SetFriction(0.5)
beam_material.SetRestitution(0.0)






def motor_torque(t):
    return np.sin(2*np.pi*t)






app = ChIrrApp(system, 'Beam Buckling Simulation')
app.SetCustomCallback(lambda t: print(f"Time: {t}"))
app.AddLight(ChVectorD(1, 1, 1))


system.SetSolverType(ChSystem.SolverType.EULER)
system.SetTimestep(0.01)


app.SetCamera(ChVectorD(5, 5, 5))
app.Run()