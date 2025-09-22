import pychrono as chrono
import pyirrlicht as chronr
 import math

sys = chrono.ChNSC()

sys.SetGravitational(chrono.ChVectord(0, -1.62, 0)  

ground = chrono.ChBody()
sys.Add(ground)
ground.Set(True)  
ground.Enable(False) 


yl = chrono.ChShapeCylinder(0.2, 0.4  
ground.AddShape(y, Chram(ChVector(0, 0))

pend = chrono.Body()
sys.Addpend()
pend.Set(False) 
pend.Enable(False) 
pend.Set(2  
pend.Inertia(chrono.ChVectord0.4,1,5 1  

yl = chronoShapeCylinder(0.1,1  
yl.SetColor(chrono.ChColor0, 0  
pend.AddShape(y, Chram(Ch(0, 0))

pend.Set(Ch3(1, 0 1)  

rev = chrono.Spherical()
rev.Initialize(ground, pend, Chram(Ch(0, 1),  
sys.Addrev

vis = chronr.ChVisualIrr()
vis.System(sys) 
vis.SetWindowSize(1024,768 
vis.Set('Pendemo demo' 
vis.Initialize 
vis.Add(logo(chrono.Getfile('logo'  
vis.Addskybox 
.Addlights() 

log = True
while vis.run():
    Begin() 
    vis() render
    vis.End()
 sys.Step(1  

if log sys time >1:
    pos = pend.Getposition() 
 print(" t", time)
 print(pos.x,  y)
vel = pend.Getvel() 
 print(vel.x, y)
log False









python