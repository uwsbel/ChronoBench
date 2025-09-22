from pychrono import ChSystem, ChBody, ChMaterialSurface, ChContactMethod
from pychrono.irrlicht import IrrlichtVisualizer
import pychrono.postprocess as postprocess


system = ChSystem()


terrain_mat = ChMaterialSurface()
terrain_mat.SetFriction(0.8)  
terrain_mat.SetRestitution(0.2)  

terrain_body = ChBody(mass=1e6)  
terrain_body.SetPos(ChVectorD(0, 0, 0))
terrain_body.SetBodyFixed(True)  
terrain_body.SetMaterialSurface(terrain_mat)

system.AddBody(terrain_body)



kraz_body = ChBody(mass=1000)  
kraz_body.SetPos(ChVectorD(0, 0, 1))  
kraz_body.SetBodyFixed(False)  
kraz_body.SetMaterialSurface(terrain_mat)

system.AddBody(kraz_body)






vis = IrrlichtVisualizer(system)
vis.SetCamera(ChVectorD(5, 5, 5), ChVectorD(0, 0, 0))  
vis.SetAmbientLight(ChColor(0.5, 0.5, 0.5))  


while True:
    system.DoStepDynamics(0.01)  
    vis.Render()