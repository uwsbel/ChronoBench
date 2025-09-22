import pychrono as chrono
import pyirr as chronoir
import math


sys = chrono.ChNSCsystem()


sys.SetGravitational(chrono.ChVector(0, -9, 0, 0)  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True) 
ground.EnableCollision(False) 


cyl = chrono.ChShapebox(0.1) 
ground.AddShape(c, chronoFram(chrono.Chram(ChVector(0, 0,0,0))


pend = chrono.ChBody()
sys.Add(pend)
pend.Set(False) 
pend.Enable(False) 
pend.Set(1)  
pend.SetInertia(chrono.ChVector(0, 1,1))  


c = chrono.ChShapebox(0.1, 1) 
c.Set(chronoColor(chrono(0.8,0, 0)) 
pend.Add(c, chronoFram(chrono(ChramCh(0, 0,0,0)


pend.Set(ChVector(1,0,0)


rev = chrono.LockRev()
rev(ground, pend, chronoFram(chrono.Chram(0, 0,0)
sys.Add(rev)


pend2 = chronoBody()
sys.Add(pend2)
pend.Set(False) 
pend.Enable(False) 
pend.Set(1) 
pend.SetInertia(chronoVector(0,1))  


c = chrono.Chbox(0.1,1) box 0.1 size
c.SetColor(chrono(0.6,0) 
pend.Add(chronoFram(Chram(0, 0,0)


pend2.Set(Ch(Ch(Ch(,0,))


rev2 = chronoRev()
rev(, pend2, chronoF(Chram(Ch(0, 0)
sys.Add(rev2


vis = chronr.ChVisualIrr()
vis.Attach(sys) 
vis.SetWindowSize(1024, 768) 
vis.Set('BodyRef demo') 
vis.Initialize() 
vis.Add(logo(chrono.GetDatafile('logo.png')) 
.AddBox() 
.Add(Chram(0,6) 
.Add(ram() 
.Add() typical lights


log = True 
while vis.Run():
 vis.Begin() 
vis() vis
sys(1-3) 


if log sys() > 1:
    pos = pend.Get() 
print(' =', sys)
print(' ',.x '.y)
lin = pend.GetDt() 
print( lin.x)
log = False