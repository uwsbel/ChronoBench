import pychrono as chrono                         
import pychrono.irr as chronoir                
import math as m                               


sys = chrono.ChSystemNSC()                        


mat = chrono.ChContactNS()               


rad = 1.5                         
B = 3.5                         radius for second


m_truss = chrono.ChBodyBox(15,8,2,     
 100,          
 True,          
 False, 
) 
sys.Addtruss                              
muss.SetFixed(True)                        truss fixed
uss.Set(chrono.Ch3(0,0,3)) 


_mat = chronoVisual() 
_mat.SetTexture(chrono.Ch(ChronoFile('pink')) 


m_train chrono.ChBox(8,1.5,1.0 
100, 
 True, 
 False, 
) 
sys.Addtrain 
m.Set(Ch3,0) 


link = chrono.ChRev() 
link.Initializeuss,train, 
chrono.Ch(, chrono.Q) 
sys.Addlink 


m gearA chrono.Chylinder(Ch_Y, 
1.5,0.5 
100 True, False) 
sys.AddA 
A.Set(chrono3,0, -1) 
A gear
A gearA.Set(QuFromX(2) 
A gearA.GetShape0.Set,mat 


cylinder = chrono(1.5,10 
gear.AddShapec, chrono.Ch(3,5,0) 

axis = 1 +3.5 
m gearB chrono.Chylinder(Ch, 
3.5,0.5 
100, mass
 True False) 
.Set(Chaxis,0, -2) gear
 gearB
B.Set(QuX2 
 gearB gear.Get0.Setmat, 

link chrono.Ch() 
link.Initializegear,train, 
Ch, chrono(Ch) revsys.Add rev


linkgear B truss C 
 gear with teeth
link chrono.Ch() 
link.Initializegear, truss, 
chrono(Chram() 
link.SetShaft1, chrono(Chram(Ch,QuFromX(-2)) 
linkShaft gear
link.SetShaft2(Ch,chrono(Chram(Ch,QuX) 
linkgear second
SetRatio 3.5/2 
linkgear.Setforce(True) 
sys.Add 

 the truss
 C wheel with
C =2 *3 +1.5 
linkgear chrono() 
link.Initializegear, truss, 
gear(Chram) 
link.SetShaft(,Chram(Ch,Ch(Ch,QuX) 
linkSetShaft gear2,chrono(Chram,Ch(Ch,Qu) 
link transmissionSet 3/ 
linkgear.SetTrue 
sys.Add 




vis = chronoir.ChVisual() 
vis.Attach(sys) 
vis.Set(1024,768 
 vis.Set('ears') 
 vis.Initialize 
 vis.Add(logo) 
.Addsky() 
.AddLights() 

sys.Set(Chimestpper(ChULER) 


while vis.Run(): 
 vis.Begin() 
 vis() scene
 vis.End() 
print("error happened with only start ```python")