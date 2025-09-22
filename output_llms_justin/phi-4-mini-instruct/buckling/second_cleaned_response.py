```python
import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"


sys = chrono.ChSystemSMC()


L = 1.2  
H = 0.3  
K = 0.07  
vA = chrono.ChVector3d(0, 0, 0)  
vC = chrono.ChVector3d(L, 0, 0)  
vB = chrono.ChVector3d(L, -H, 0)  
vG = chrono.ChVector3d(L - K, -H, 0)  
vd = chrono.ChVector3d(0, 0, 0.0001)  


body_truss = chrono.ChBody()
body_truss.SetFixed(True)  
sys.AddBody(body_truss)  


boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))


body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  
sys.AddBody(body_crank)  


boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)


motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  
myfun = ChFunctionMyFun()  
motor.SetAngleFunction(myfun)  
sys.Add(motor)  


mesh = fea.ChMesh()


beam_wy = 0.12  
beam_wz = 0.012  


minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  
melasticity.SetShearModulusFromPoisson(0.3)  
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  

msection1 (100 (1 and 7
1
1 (1 (0 and 1 and 9 and 3 and and 1 0 (1
3
1 and 
 and 12 and 2 (x
2 3 3
f. and * 1
1 1 0 1
 2 and and 2 in the in *l 1 * 1 * 1 *  2 1 * 1 * 1 12  2 (1 8 (2 * 2 and 1 and 1 and * 1 2 * and and 3 (o (1 1 (o (0 1 1 and 0 0 0 * 1 * 0 * 1, * 1 and 0 2 1 0. (1 and 1 and 1 0 0  (1 2 1 and 2 (1
 1 0 0 * 1 0 0 * 1 0 1 and 2
 and and and and and and and and * and 0 **0 and and 0 0 and 2 ***0 and "and and and and and and and 0 and and and 4 **  * 2 1 * 1 4 * 2 * 1 *0  and  * * * 2 * 1 *x 2 (2 * and and * 2 and * and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and 2 and * 0  and and 2 1 1 * 1 2 2 * * and * (l 2 * 2 * and * (2 and and and and " 2 2 * * 2 2 2 2 and 0 4 2 2 0 0 2 2 (1 "1 2 0 2 2 (2 7 2 and (f 0 (2 * 1 2 * 0.  1 (2 12 and and and for for 2  and 1 2 (4 1 2 1

10 2 1 and 3 and "  and 2  
2 and and and and and and and and "2 and "2 2 1 and 2 * and "x
 and 2 2 2 * 12 23 and " * 2 * * * * "1 2 2 2 * 1 * " * and " * and (2 and and and and "2 and and and  2 and and and and and 1 and " * "2 "  " and and  " and and and * * * * to to * * 2 2 * 2 * " * 2 1 * 0 * 2 * 2 * " * 0 * and * 0 * (0 1 1 2 (2 0 * and and and and 2 and and "100 2 (1 (1 2 2 2 1 and and " and 2 ( * and 2 2 (in 1 2 * 2 0 *  and and and and and and and and and 2 2 2 3 0 2 (5 1 * 2 ( 0 2 2 0 * " 2 0 2 * 21  2 2 * (0 0 2 * (0 (3 (2 7 ( * (2 2 2 * * and *

. (1 2 (1 *  * 1 0 2  * 1 () 2 (1 1 0 " * "2 2 2 2 0  *  2 1 2 2 2 *
 *
 1 and and and and and 0 and and and and and and and and and and and 0 and and * (2 and and and and and 0 and and and 1 1
 2 * and 0 * (2. 1 2 2 * 2 2 *2 2 and * 1 * and 2
 * and and and and "1  *

0 2 and and and and and 2 and * "0 0
 * 2 2
 2 2
0 * " 2 2  and and * and  and * and * 2  * ( * 2 * and *
2 *  
 1 2 1
* *
1
 *
 2 2 2 4
 2 12 2 * 1 * and * * 2
2
2 4 2 2  2 and and and 2 2 2 0 2 2 * 2 4 2 2 2 2 2  
 and and 2 * 0
  4 2
t * (2 2
 4
   4
 * 2 2
 2 * 2
 2
2
2
 2
2 (2
2. 0
 2
2
1
2
2 * 2 and 1.
 * 2 * and 2 and 2 * and and and 2
1 * (0
1 2 and and and 2 * and and and 0

and and * 0 * * 2
1 2 * 2 * 2 2 * 2 * and * * 2 * 3 * 2 * that * 2 * 2 * 4 * 2 and and and * 2
2 * 2 * * * * and in  
2 * 0 *2

12
 2
 1 2


2
2
2 2 2 2 1 2 * and and and and 2
 and and 2
 * and and and 2
 for 2 2 2 2 for 2 * 2
1
2, 3 *2 * (1 2 2 * 2 2 2 2 1 * * and * 2 2
 *
0 2
1 * 2 2 * 2 2 2 * 2 1 * * 2 2 * 2 (1 * 12 9 * for 4 0 1 1 (1 2 1 1 * 2 2 2 * 2 2 and and and and and and " 2 2
 *  0 * and and and and * 0 and 2 0 2 * 2 * 2  2 2

 2 2 2
 2 2  (in 4

 * 2 and and 2 * and 2 2 * 2

 2 and and and and 2 2 and and 2 2 and 4 1 and "0 2 * 2 (2 and 0 2 *


2 and 2  * * * (1 * 0 and * 2 * * 2 1 * 2 2 and and 2 2 2 * 2
2 2 * 2 2 2 0 * 2 and "1
 and "2 * 1 * 2 (2 (2 2

2 2 2 0 2 * 2 *0 *2 * 2 (0 * 2 (2 * 2 (5 2 (4 (2 1 * 2 for  for 2 4 2 2 2 2 0 in 2 (2 2 2 for 2 2 2 2 4 2 2 2 2 1 2 2 2 2 (2 2 2 2 2 2 1 (3 2 2 *2 * 1 0 0 (2 2 2 2
1  0
1 in 2 1 2 (1 2
2 2 (2 2 2 for 2 1 2 2 and 0 and 2 and and and and and and and and and and and and and and and and 2   and 2 2 (0 and 0 7 2 2 1 0 (2 2 * and 2 in 2 * and 2    and and and 2 2 * 2   and and and and and * and and and and 2 (2 and and 0
 and and 2 *2 2 2 (2 0 * 2 2 (1 *2.0 2
2 2 (2 and 0 *2 * 1 (1 (2 *2 2 2 2 2 2 for  for for 2 4 2 for 2 2 2 2 2 2 2 2 2 0 2 and  for  for 2 0 0 for 2 2 2 * 4 * 2 * 0 for 0 for (4 *  for 2 for 2 2 2 (0 2 and * 2 1 for 2 for 2 for 2 for 2 1 0 for 2 2 2 2 for 2 0 * to *2 (2 2 and and and and and and and and and and 2 and and and and and and and and and and 0 * 2 * 0 * 2 * 0 * 2 2 0 * 2 *  * * * 2 2 2
 2 * 2 * 2
2 * 42 (0 * 2 4 2
 and  * 2 * 2 3 2 and and and and 2 and 2 2 and 2 and     and and (2 (2 * to and "1 and * 2 (2 * * * 2 *  * 2 *  * 2 * * and (2 * * *2 * 2 and and and and and and and and and 2 2 * 2 * and 2 2 2 * 2 2  2 (1 0 2 2 * 2 * (1 2 * 2 2 *2 2 in 2 (1 * and the 2 * (0, * 0 2 2 2 1 2 0 * "2 0 0 for  for    2 * 2  * 2
1 *    *   0 (1    in 2 and 2  in * and  1 0 *  0  and 2 in  in  and and  1 and and and and *  in * 2 * * * 2 * 2 2 * 1 * 2 * 2 2 * 2 * 2 * * and 2 * 0 * 2 * 2 * and 2 * * 2 * 2 and and and and and 2 and and and and and and and and and and and and and and and and and 2 * 2 ( 2 * 2 * 2 ( * 2 2 * 2 and and and and and and and 0 * (1 * 2 and and and and and 2 * and * * 0 * 2 and * 2 2 * 2 * 2 * 2 *2 * and 2 * 2 * 2 * * * 2 * * * * 2 * 2 * 1 * *2 and and and and and and and and and and and * 2 and and and and and and and and and 2 * * 2 2 0 3 and 2 2 * 1 1 *  *    0  2 * 2 * 0 * * * 2 * 2 * 2 * 2 * * * and  * * * and and and * and * and and and 0 * and 2 2 2 2 2 (3 1 2 2 2 2 2
2 2 *
2 * * * * 2 2 * 2 2 2 2 0 2 * 2 2 and and and 2 and and 2 2 2 2 2 and and and and and 2 and 2 2 2 2 2 * and and and 2 2 * 1 2 2 2 2 2  * 2 2 1 and 2
  2 2 2
1
2 2 and and and and 2 and and * 2
 * 2 2 2 2 * 2  * 2 2 *
 *   * 2 2
2
 to 2 * 2  * and 2 2 2 and 2 and and and and and and 2
 * 2 2 to 2  * 2 2
2
2
 2 2 2 * 2
2 * 2 2 1
 *2 * 2 2 * 2 * 2 2 2
 2  * 2 * 2 *  
 2 2 2
2
2 * 2 2
2
 2 2 2 * 2 2 2 2 and 2 2 2 2 2 2 * 2 * 2 2 2 2 2 2 2 2 2 and  * 2 2 2 * 2 * 2 and 2 2 2 * 2 * 2 2 2 2 1 0 2 *2 0 0 * 2  2 2 1 2 * 2 * * 2 and 2 * 1 * 2 and and and and 2 and * and 2 1 
 2 * 2 * 2 * 2 * 2 2 1 and and and 2 * 2  *  and 2 and 2 2 and 2 2 2 and 2 and and and and and and and and and *2 * 2 2 and 2  * 2 2 2 2 2 2 1 2 2 2 * 2 2 2 2 2 2 2 * 2 2 2 0  2 2 * 2 2 2 2 2 and 2 and and 2 and 2 and and and 2 and and and 2 *

 * 2 1
2 2 2 2 2 * * * 2 * * * 2 * 2 0 2 2 * * 2 and 2 2 and 2 1 2 2 2 * 2 2 2 0 0 2 3 2 2 2 2 2
2

 1 2 2 * 2 2
2 22 2 2 2 2 2 2  1 2 2 2 2 2 2 2 1 2 and and 2 2 * 2 1 * 2 2 2 2 2 2 0 2 2 2 2

 12



 * 2 2 * 2 * 2 and and and 2 * 2 and and and 0 and and and and 2 and 2 * 2 * 2 * 2 * 2 and 2 * 2 * 2 * * 2 * 2 * 2
2 2 * 2 *2 and 2 and and *1 and and and 2 * * and and and and and and and and and and 2 and and and and and and and and and and 1 (3 2 1 2 * 1 2 2 2 0 2 * 2 2 2 * 2 * * * * * * 1 * * 0 2 2 (2 (2 2 2 * 2 * 1 * 2
2 (2 * 2 and and * 1 * 2 and 2 for 2 for 2 2 2 for 2 * 22 for 2 (2 and and for 2 and 2 * 2 and and and the 2 and and and and 2 and and and 2 and and and and and and and and and 2 (2 2 2 2
2 2 2 2 * 2 2 2 and  * * * 2 * * * * * * 2 2 and and and 2 * 2 and and and and and and and and and 0 *  * 1 *  and and and 2 and and and and 2 * 2 and 2 and and and and 1 1 to 2 *2 2
2 and and and and and and and and 2 and and 1  and and and and  and 2 and and and and and and and and and and and and and and and and and and and and and and 2 and and and and 2 2 2
2
2 2 2 * 2 *  * 2 * 2 * 1
 * 2
2
3
2 2 and and and and and and 2 and * *
 * 
 2 *
 and and 2 2 2 2 2 2 2 2 * * 2 2 * 4  2 2 and and 2 2 and 2 0 and and and and  and and  and 2
 *

2 and 2 2 2 2 2 2 2 8 * 2 * and 2 * 2 * 2 2 2 2 and 2 2 2 2 2


 2 2
 * 2 2 2 and and and and  and and and and and and and and and and and and and and and and and and and 2 and 2
1 and and 2 1 1 * 1 *  (1 2
2 2 1 1
  2 2 2 2 and and and and and  1 1    in  and and and and and and and and 0 and    and 2 2 2
 and 
 2 2 2 2 2
*2 1 2 in 2 * 1 2 and and 2
2
1
 * in 2 and and and 2 and 1 and 1
 2 and and and and and and 2 2 2 and and 2  and and and and and and and and  2 2 * and 2 
* and and and 2 2
2 2 2 and and and  and  and and 2 * 2 and 0 2 and and 2 1 2 2 2 and and and 2 2 2 * 1 2 2 *   2 0 2 2 * 2 2 2   2 2 7 * 2 and and and and and and and and and 2 2 and and and and and and and and 2 2
1 2
2 2 2 2  and and 2 2  in 2 2 2 2
2 2 2
1  and and and and and and and 2 * 2 and 2 *  and and and and and and 2 and and and 2 and and and and and and and and and and *
 *
 and and and and and and and and and and  and 2 2 2 and and and and and and and and and and 2 2 * 2 and and and and and and and and and and  and and and  and and and and and and and and and 2 and and and 2 and 2 2 1 2 2 2 0 and and and  and  and 1 2 2 * and 2 2 and and and and and and and and and 4  and and and and and 0 and and 2 and 12 and and 0 and  and 2  in 1 and 2 2 0
b
2
 * 2
10
/*
 2 12  and and and 1 2 1  2 2 2 2 2 1 1 1 2  and and and and and and 1 * and 2 0 2 2 0 12 1
 2 in 2 7 for 4
 2 * 2
2 0 2
 2
2
2 * 0 2
0
2 * and 2 2 2 and 2 2 2 and and  and  and and 2 2 for  for 2 2 and and and 2 2 2 1 3 2 and 5 and and and and 1 and and and and and and and and and and and and 2 and and and and and and and and and and and and  and and and 2
2
2 2 0 and 0
2 * 9
 *
2 and and and 4

 * 1 * 2
 2

 and and and  and 2 and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and 2 * 2
1
1 2
 and and and and and and 2 * * 2 1

 and and and and and and and and and and  and and and and and and and 2 and and and 2 and 2 and 2 3
2 2
2
2
0 2 2 0
2 2 2 *1 and 1 2 2 0 1 2 2 2 2 2 2  and and and 2 *0 2 2 0 and and and and 0 1 and 4 and and and and and and 2 and and and 2 0 * 2 7
2 and and and and and and and 2 2 and and and and and 2 and and 2 12 0 and and and and and and and 8 and and and and


2 and and and and and and and and 2 *2
2 * 1 2 and and for 2
2 2 2
2 2
2
2 and and 2
2 and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and 0 and and and *1 * 2 * 0 2 *2
2 2
2
4 * 2 * 2
 and and and and
* 2 1 * 2 * and and and and and and and and 2 2  and  and 2
 and 2.0
2 in 2 2 and 2 and 0 and and 2 2 2 2 2 for 2 and 2 2 2 for 2  and and and for 2 and and 2 2 and and * 7 and and and and 22 2 and and 2 2 and and and and 2
2
2 0 2 2 4
20
8  2 2 *2
1
3
2 2 2
2
 and 2 2 *  and and 2 2 * 2 2 2 and 2

1 2
2

 2 and 2 0 1.2
2
2 2
2 2 2
2 * * and 2 * and and and and and and and and and and 2 and and and and and and and and and and and and and and and and and 2 and and and and and and and and and and and 2 and *  * 2 * 2 2 2 2 *2

 2 0 0 * and and and 2
 * 2 2 *  and and and and and  and 2 0 and and and and and and and and and and and and and and and and * 0 and and and and and 2 and and 2 0 0 * * * * 2 * 0 *1 2 *2  and and and and and and and  and and and * 2 and and and and and and and and and 2 * 2 7 0 4 * 2 0
2
2 0 2 * 2 * and * 4 2
 * 2
3  and and and * 2 2 * 2 * for 2 2
 for 0 for 0 * 2 22 0 and and 2 2 2 2 2 * 2 and and 2
 2 4 0
2
1 2 and 4 0 * 2 * 2 1 2
1 * and and and * * * 0 and and and and and and and and and and and and and and and 0 and and and and and 2 and and 0 * 2 * 2 0 * 2 * 2 * 2 9 2 2
2
2
2 *0 * * and and and * * and and and and and and 3 and and 2 and and 0 and and and and and and and 2 * 2 and 2 * and and and and and 0 * and * 3 * and and and and and and and and and and 2
*100
 * and * and and and and 2 * * and 2 1 and and and and 0 * and 1 2
2
1 * and in 2 and 1 * 2
2 * 1 2 2 * in 2 2 0 2


l 2 0 0
2 2 and 0. * and 0
9
8 8.2
2 2 and and and and and and 0 and 2 2
0

2 2
1
2
2
1
2 2
4

 4
1
in
 * 7
2
 * and
l
1
2
1
2
1
2
0
 and
2
*

2
9
4
2
2
1
f
1 0
2
2


2
2
6
4
1
 2
y * 2
2
2
2
 5 * and 3
 and and and 0
0
2
3
 2 and and and and and and and and 2 2 and and and and and and and and and 2
2 and in 2
 and and and and 0 0 * 2
 0 2
2
2
2
2
*2 *
b
2 and and and and and and and and and and 2 * and and and and and and and 2
2 (2 and 0 1 ( and and and 0
 and in the 2.2 (2
7
 and and and and and and
 and and and and and
 and and and and and and
 * and and
0 and and and and and and and and and and and and and and and and * and * and and and and * 2 * 1
2
 3 0 1
 * 1 * 0 2 (1
0 0
2 * and for 2 in 0. and in 2 0 *  *
 * and 2
3
2 and and 0 * 0 * *2 * 0
1 * 0
 2 2 for 0
 2  from the 2 * and 9 0 in 1 *
 2 *  to  * 2
 * * * * * 1 * and and and 2 * * 2 in 0 * and and 4 2
 
 * 2
 * 2
  2
2 0 * 2 * 0
 * 2 *
  *  and 2 in 2 * and the 1 in 2 * 0 0 and 2 * 0 *2 0 * 0
 22 2 * 2 
  and and and 1 2 2
2 (2 * ( and 2 and 3 0
 * 2
2
2  and and 0
3
2 * * * and and and and and 2 * and 2 0
 and and and 1 and and and 0 0 * * 1
2
1 * 2 *  * * 1 * 2 2
 2 * and * and * 2 * 2 * 2 3
2 in 2 in 2
1 * 4 and 2 and and and and *  in 2
  and 


 2 2 0 * 0 2 2 2 * 2 2 2
2
1
2
1 2
2 0 0 * and 2 30 * 0  * * 1 0 * 0
1 0 0 12
1 0 0 *2 2 2 * 0 in 2 * 0 *2 2 2 * 2 0 2 (2 2 2 2 0 *
1 0 3 * 0 *2  and and and and and and and and and and and and and and and and 2 and and
1 2 0
1 (0 * 2 (1 * 0 * 3
 * 0
2 (
2
 and and 0 * 2
1 *1 and and and and * and * and and *

 * 2 * * 0
 and 0 *2 * 1 * 0 * 3 2 * 2 * 2 * 2
 * 0 * 0 * 0 4  * 1 * 0 * 2 * 2
 * 4
2 * * and 0
* and * 0 * * * * 0 * and and and * 0 and 2 * * * * * and and and * * and * *
 * * * * 2 * *0 * 2
 * 0 123
2 * * 1 2 * 1 * 2
 * 2 * * 1 0 123 2
2
2 2 2 * 0 * 1 * 3
* 0 * 2 2 0 * 0 * 1 2 * 2 * 8 *2 4 * 2 2 * and *2 * to and and and and 0 2 2 * * * * * 2 * * 2 * 0
 * 2 *
1 2 1
2 * 2 * * * * 0 1
 * 3 *
 *2 *3 *2 * * 0 * 2 * * * *
 * 4 *0 2 1 2 and 2 *2 * and and and 2 and and and 0
 *
 *
 2
 *
2
4
1
2
2 * 0 * 2
 * 2
 * * 1 * * *1 *4 1 * * and * and and and * 0 * and and and and 4
 * 2 * 0 * * * 2 * * 0 0
2 * * * *  * 2 * 4 *2 * 0 * 5 * 2 * 2 * 2 *  * * * * * * * *2 * * 2 * 2 * 2 0
* 2
 * 2 * 2 * 2
 * 2 * 2 * 2
* 2 * 3
2 *2
2
 * 2 * 2 * * 2 2 * 2 2
2
2
2
 * 2 * 2 * 4 * 4 *  * 1 * * 2 * 2
 * 1 * 0 * 0
 * * 2 * 4 * and 2 * 0 * * and * 0 2 * 0 * 2 * 0 * 2 and 0 0 * 4 * 4 2 2
2
 * 2 2 * * * * * 1 2 2 * 0 * * * 4 * * *  * * 0 * *  * * * * * ** * * 0 * 0 2 * * 0  * 2 * * 2 * * * 1 * * * 0 2
*2 * 2 * * *  * 1
 * and 2 * * 2 * * 0 * 2 * * * 2 * * * * * 2 * * *2 * *  * * * 2 * * 2 * * * * * 0 * * *  *  * *  * *  * * * * * *2 * * * * 2 * * 2 *2 * * * *2 * * * *  * * *0 * 1 * * * * * * *4 *2 * * 2 *  *2 0 *2 *2 *3 *
2 2 *2
 * 2 4 2 for 2 2 * 2 2 * 2 * * *2 2 2 0 0

* 2
2
2 2
 * 2 2 * 2 * *
 *
 * * * to 4 2 *0 * 2 * 2
* * and * 2 * and 0 *2 * 0 4 * * 1 * 2 * * 0 * 1 0 * *1 *1 * 1 *2 *4 *2 * *0 *  * * 2  *2 * 2 * 2 2 * * * 2
 *
2 2 * 2 2 * * *  * * * * * 2 * * * 2 * 2 * * * * 2 and and and and and and and and and * * * * * * * * 2 * 2 * * 2 * * * * * 0 * *
 * * * * and 2 *2 *2 * * 4  * * * * *  *2 *0 * and 2 * * * * * 4 *2 *2 * 2 * 2 * 0 *0 2 * 2 * 0 * * * 2 * * 2 * * 2 * 0
2 * 2 for * * * and *  for * for 3 0 * * * * 3 * 0 in 0 2 * and * and * and * and and * * and and and and and 2 * 0 * * 2 * 1 * * 2 * 2
* 0 *2
3 0
 * 2
2
2
1 * * * *
 * * * * * * * * * * * and *2 * 2 * * 0 * 0 * 2 * 1 * 0 * * * * * * 2 * * 0 * 2 * 2 *  * * * * * * * * and and and and * * and 2 *0 * 2 * 0 and and and and 2 * * 2 * * * * * * * 0 * and * * * * * * 2 * * * * * * 2 * * * * * * * * * * 2 * 0
 2
 * * that 0 * * and * and * * * * * * * 1 * and and and and and and and and * and 2 * and and and and and and and and and * * * * * * * * * * * * * * * * * * * 2
 * * * * * * * * * * * * * * * * * * * and * and * and * and * * * * * * * 2
 * * * *  * * * and * * * * 0 2 * 2 * * 2 * * * * * * * * * * * * * and 2 * * 2 * * * *2
* * * * * * 2 * *  * and 2 * 2 * * 2 * * * * * and and and the
*  * * * * * * * * * 2

 2 * 4 0 * 2 * * * * * * * * * 2 * * * * * 2
 2 * 2
 * * * * * * and * and and and and and * and 2
2 *2 * 2

 * 2 * * * 2
 2 * 0 * * and 2 1 * 2 *
 * and * and 2 and and * and * and and and * and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and and * and * and and and and * and * and * * * * and and
 and * and and * * * * * * * * * * * * * * * * and * * * * * 2 * * * * and and and and * and * * * 2 * 2 * and and and and and and and and and 2
 * 2  

 1 * and * *0 * and * 2 2
* 0
 * 1 * * * 2 * 1
 * and  * 1 * * * * * 0  * 0 * 0 2 * 0 * * 2
2 0 * 4 * 2 * 0 *  * 2 * * 2 * 1 * * 0 * * 2 * and 0 * * 0 * * 2 *  * and and 2 * * * * * * from  * 2 4 2 * 2 * * * 2 and 2 and * and * 1 * *  * * * * 2 * 0 * 2 * * 2 * * 2
 and and and and and  and and and and and and 1 and and and and and and and and and and and *  * and *  * and * and and 2 * * * 2 * *  * * 0  *
 *
 * * * * * * * * * * 24
 *2 * * * 0 * * 2 * and * and * and * * * * * * * *
 * and and and 2 * *  * 2 

 * 2
 *  * 2
* to 2
1 * * 2 * * 2
 * 2
 * and * and 1 * and * * * * * * *  *  * 3 * and 0 * 2 2
 * * * * * * * * * * * 2 * * * 2 * * 2
1
1
 * 4
0
 2 * 0 * 2 * 2 2 *2 * 2 * 12 * * * * * 4 * 3 * and *  5 2 * and * * * * * * 0 * 4
 * *  *  * *  * 2 * * 2 * * * * * 1
 2 * 2 * and * and 0 * *2 * * * * * * and * * *  * * * * * *  * * * * * * * * * * * * * * 2
1 * * * * * * * * 0 * * * * * * * * * * * * 4 * * * * *  * 2 and 2 * and and and 2
 2 * and and and and and and and *  * * * * * * 2  and and 2
 and and * * * 0 * * * * * * * 1 * * 0 * 0 0 * * 0 * 0 * * * * *  * * * * 0 * and * 1 * 0 * 0 * and and * * *  *  *  * * * * * 1 * and
 *  * 0
 *
*0 * * * * * * * * * * 1 * * 2 * and
 and and and and and and * and 2 and and and and * and * and and and and and *  * 0 * 2 2 2
 * * 0 * * * * 0 * * * and  and 0 * 2 and 2 * 0 and
 and 0 0 2 * * * * 0 * and * * * 0 * 2 * 3 and and and and and and and and
 * 0 * 0
 for 
 * * 2 * 2 3 * 0 * 2

 *  * and * * 1
 * * * * * 2
 * and 2 and and and
 * and  and and and
 * and and
 and * 2 and and and and and and and * and * * * 0 and
 * 3
 * 2 * * * * * * * * *  * * * 2 * * and  *  * and  *  * * and and and  * * and  * * * 2
* * 2 2 * and * * * * * * * * * * * * * **
 *  * * * * * * *  * * 2 * * 2
 * 2 *  * 2 * 2 * 2 * 2 * * * * * 2 * and * and  for 2 *  * 2 *  *  * 2
 * 2 2
* * * for 2
 *  * * for  *  * 0 * 2 and  * * * 1 * 2 * *  * and and * * * *  * * * * * *
 * *  * * 2 and * 2 * * for  * 2 * 2 * * * 2 * * 2
 * 2 * 2 * 2 * * 2 * * 2 * and 0 1 * and * 1 * * * * 2 * * 2
* * 2
 * * * * * 




 * * * * * * 0 * and * * * * 2 * * * * * * * * and * 
 * * * * * * * and, 
 * * * and * and  and * * * * * and and and and and and and and and and and and and and and and and and and and and and * *  *2 * * * * 2 * 2 * * * * * * * * * * * * * * and and and and and * and and and * *0
 * and and and and and and and and and * and * and * and  * and and and 0 * *
 * * * 0 * * * * * * * 2 * * * 0 * 0 2
 * and and the * * * * * and  * 0 0
0 * and 2 * * * * * * * * * and * * * * * * * * * 2 and and and and  * and * and the * * * * 2 * * 0 2 * *  2 * 2 *  * * * 2 * 1 * 2 * 2 * and 2 * * * *  * * *  * * 2 * 2 * * 3 * * 2 * 0 * * * 2 * * * * 2 * * 2 * * * 2 * 2 * 2 2 * * 2 * *  * and and and and and * and and and * * * and and and and and * and and * * * * 3 * 2 0 2 * * 2
 * * * * * 0 * * * * 2
 * * 0 2 2 2 * and and * and * 2 * *
 *  * * and 2 and and and and * 2 * and and and and and and and and and * * * * * * * * * * and * and * 2 and * * * * * * * * * 2 * * * and * * * * * * * * * * * 2 * * * * 2 * * * * * * * and and and and * 2 * and the * * and * and and and and and and * and * * *  * * * and 0 * * 2 * 2
 * * * * 2 2 2 * * * 2 * * *2 * 2 and and and 2 * * 2 2 * 2 * 2 and  for the * 2 * * for 2
 * * 0 * for 2 * * * * * for 2 * * * 2 * * * * * * *0 * 2 * and the and and and and 2 and 2 and 2 * and and and and 0 and and * 2 * * and and and and  and and and * and 2 and and and * 0 * and  * 2 3
 0 * * 2 * * 2 * 1 * 2 * *  * 2 and  and 0 and 2 * * * * and and and and and and and and and and and 1 and and and and and and * and 2 and 2 * and * * and 0 * and and 0 * and * and * * * 2 * *  and and and and  and and and 2 and 2 and  and and and and and and and 2 * * * 0 * * 0 1 and  * 2 * *  * * * * and * * * and * and * * and * *2 * * * * and 2 * * 2 * * 2 * * * 2 * and 2 * * 2 * 2 * 0 *  * and * 2 *  *  * 2 * * * * * * * * 2 * 2 * 1 *  and 2  2 * to 2 * * 2 * * 2 *  * to 2 * 2 * * * and * and and and and * and * * and 2 * 0 * and 2 0 2 2 * 2
 * 1 * * * *  * 2 * * 2 * 3 * 2 * 2 *  * 2 * 2 2 * 2 * and and 2 * * 2 *  *
 and
 and and and and and 2 and 0 and and and 1 and and and and *  *
 * 2 * * * * * 2
 * 2 * * * * * * 2 * *  * and  * * * *  * 2 * 2 * * 2 * * * * * * * * 4
 * * * * 2 * and in * * 2 * * and 2 * 2 * 
 * * * *
 * and * *
 * and * 2 * * * * 1 * * and 2 * and * and * * * * * * 2 * 2 * and * 2 * * *  * * 2 and 2 * 2 * * * * * and 2 * 2 * 1 * * * 2 *  * * *  * * * and and *  * * and and 2 * and and and 2 * 2 2 * 2 * 2 *  * 2 2 * 2 * 2 * * * 0 2 * and 2 * 2
 and and and and and and and and and and and and and and and and and * and and and and and for 2 and 2 * * * and and and and and and 4 and 2 2 *  to and and and 2 and and and and * 2 * * * 2 * 4 * 2 * * * 0 * 2 * * 2 * 2 *  * 2 and * 2 and and and and and * 2 and  and * and the * 1 and 2 and and and *. * 0 * and * 2 * 2 in 2 *  * * * *
 * * and * 
 * 2 * and * 2 0 2 * 2 and and 2 * * 2 *  * * * and * and *  2 0 2 1 in 2 * 2 * *  * * 2 *
 and the 0
 * 2 * 2 *
 * 0 * 0 2 * 3
2
 * in
 * 2 * 2 * and 2 * 2 * * 0 * *  * * 2 * (0 for the * 2 * 0 2 * 4 * 2 * * * 2  *  * 2 * 2 * 0 * 0 * 1
 1 * * and * 1
 * 2 * * and and and 0 and * and and 1 0 0 0 2 0 0 * *  * 0 2 * * * * * 2 0 * * * * 2 * 0 * 2 *  * 2 * 2 * * 2 and and and 2 * and 0 * * * *  *  * 2 * 0 * 3 * * * * * * * * * * * 0 * 2 * * * *  * *  * * * * 2 * * * 2 * * * * *  * * * * *  * *2 * 2 * * * * * * 2 1 * * * * * 1 * 1 and * 0 *  *  * *  * * * *  * *  * 2
* * 2
 * 2 *
 * * 2 *  * * * and * and * and and and * * * 2 * 2 1 * 2 * 2 * * *  * * * 0 * * * and * 2 * * * 1 * 0 * * 2 * * * * 2 *  * and and and and and and and and and and and * 4 0 1 2 0 *  *  * 0 *  * 2 * 2 * 2 * to 1 *0 * * * * * * 4 * * 1 * * * 2 2 *  * 2 * * 2 * 2 * 2 * to and and and and 1 0 * * to 2 * and 1 2 0 2 2 2 * 2 * 0 2
 * 4
2 * 2 1 2 2  * 2 2 0 2 3  and 2 2 0 and 2 2 2 0 * and 2 * 2 * 2 * 2 * * 2 * 4 * 1 2 * 1 * 0  * * * * 2 * * 2 * and 2 * and and 0 * 2 *  * *  * * and and * * * * * * * * * * *  * * * * 2 * and  * 0 *  * 2 * * * * * * * * * 2 * 2 * 2 2 * 2 * 2 * 2 * 2 2 * * * * and 2 * 2 2 2 * 2 * and 2 * 2 *  * * and and and and 2 * and 2 2 * 2 2 * * 2 * 3 * * for 2 * 2 * * for 2 * 2 2 2 * 2 2 2 * 2 * 2 * 2 * 2 * and 2 * 2 * 2 * 2 to 2
*  *  *  * * 2 * 2 * 2 * * * * 2
 *  * 2
 * 1
 * and * * 2
 * 2
 * *2
 *  * 2
1
* 2
2
 * * 2
2
2
* 2 * 2
1
2 * 1

2 * 2 * 2


 1 2 * 2
2 1 2
2 * 2
  2 *  *  *  *  * * * 7 3 * 2
 2
 * and
 *  and
 * to 2
 * 2 0 2 * 2
2 * 0 * 2
 * 2 * 1 * 2
 * 2 * 1
2 2
2 * 1 * 1 2 *  *  * * * *
 * * * 2 * 2 * * * 2 2 * 1 * * 0 * * * * 1
 * 2 * and * * 1 * 2 2 * 2 2 2
1 * 2
 * * and and * 2
 * and 1 * * * 1 2 *  * * 2 * 2 * 3 * 4 2 * 2 * 2 2 * 0
*2
 * 2 * 2
 * 2 and and * and and and  * 2 * * 1 * and * 2 * and 1 * 0 * 2 * * * 2 * * 2 2 *  *  *  * * * * * * 1 *  * * 0 * * 2 * 0 * and 2 * 2 and the *  and  and and  * 1
 * 2
2 * 0 2
 * 0
 * 2
 * * 0 * * * 2 * 0 * 2 * 1 * 0 2 * 2 2 * 2 * 2 * 2 * and 2 * 2 * * and 2 * 1 1 4, * * 2
 * 2 * 8 2 and and and 2 * and the and 2 * 2 and
 * 2
 * * * and * 2
 * 2
* 2
* * 2 * 2 * * * * 2 * 2 * * 2 * 42 * 2 * 2 *  * 2 * and the 2  * * and 0
 and 2. 0 * 2 and 2
 * 2 and and 2
1
 * 2
4
 *0 * and 1 2 2 * 0 for 2 2 * to * for  for * * 2 * for 3 * * 7 2 3
2 2 * 2 for * 2 2 * and 1 and 3 * and and and and 4 and 0 for 2 for 0
* and * 1 * 2
* * to  * * 2 * 1 * 1 * 2 in the for  for * 1 * 2
* * and * 2 2 * * 2 * 2 2 * *  * and * 2 * 2 and  and * and 1
  * 1 to * to 2 * * 1 to * * * *  * * 1
 

 * 0  *  * * 0
 * 2 * * * 2
 * 2
* 0
2
 * 2 2 * *   *  * * * * * * * 2

  * 2 * 2 * 2 * * * * 2 * and and * 2 * * * * * *  * * * * 0 * 3 * * *  * 2 * for * for * * * and * 2 * 2 2
 * and * 2
 * * * * * 2 * 2 * * and * and 1 * * and and, 2 2 2
 * and 1 *  * 2
 *
 * * *  * * to 2
*0 0 * 2 * and and * 2 * and * 2
 * 0 * and * 1 * * 2


   2
 2
 1
2 * *  * * * 2
 * to the 2 * 2
* 0 * 4 * 2 * * 2
 * * and * 2
 2 * 0 * 2 * * * *  * * *  * * * * * * * * * * * * *  * * * * * 1 * * * * * * * * *
 * * * * * and * * *  * * * * * * * * 1 * * * * * * * * * * and * * * * * * * * * * * * * * * and and and 2 * * * * * 2 * * * * * *  *  * * 2 *  * * *   * * * * * * * * * * * * * * * * and   * * * * *  * * * * * *  *  * * * * * * * * and * * * * * * * * * * * 2 *  * * * * * * * 1 * for * for * * * * * * * and * * * and * * * 
 * and * *  * for * for  *  * 2
 * * 2 * to * to 2 and *  *  *  *  *  *   * 1 *  * 2
 * 2
1
 * * 2
 * *  (m 2
0 * 2
4
  * 2 2 * 0
  * *  *  *  1 * 2 2 * to
 2 * 2








 * and



 * 2
 * * 2 * 2 and 2 * 2
 2
 * and and and
 * to
 * and * 2

 * * and  and and and and and * * * and and * and * * and and * * * and * 1 * 0
 * * * and and * and 2 *  and * * * * * *

 2
* * 0 * 2
 * 0
 * 2
 * * 0 * * * * * * * and
 * 2 * 0
 2
 2 2 *  and  * * * 2 * * * * 2
 * 0
 3
 * * *  * * * * * * * * * * * *  * 0 * 0 * and and 2 * 0 * * * * * * * * * * *  * *  * * * * 2 0 * * * * 2 *  * * * 2 * * * *  * * * * * 2 * * * * and the *  *  *  *  * * * * * * * * and * and  * * * * * * * 2 * *  * * * 0 * * * * * * * 2 * and
 and
 and 2 2 * 2
 0  *  * and * 2, 2 * * and * * * 4 * and * and
 0 and * * * 1
 * *  * * * * * * * * * * * * * 2 * * * * 2 * * * * * * * * * *  * 2 * *  * 2 * 2 *  *  and 2 *  * * * and the * and * and * * * * * *  * 2 * * and the  *  *  * * * * * * * * * * * * * * 2 *  * and *  *  * 1 * * 2 * * 2 * and * * 1
 * 0 * 2 * * 0
* 2 and
 * 1 * * * 2 * * 2
 *2
 3

 and 2






 * and 2
*1
 1
2
2
*2
*2
2
*2
 2
* * 2
* * 2
 * 0
2 and 1
   1
   2 * 2 2 * * * * * 2 2 * *  * 2 * and  * * and * * * 2 * * * * 2 * *  and * 2 * 2 * 2 2 * 2 * 0 * * * * * 2 * and * * * 0 * and and 2 2 * 0  * 2 * and and  * *  * 2 * * 0 * 2
* and and the 2
 * 4
* * and and * and and and and *  * 2
* 2 and and and and and  and and and and and 0 and and * and 1 * * and 2 * 2
 * * and  * * * and and * * * * * * * * * * * and and * * and * * and 0 * 2
 * and * and *  * * * * *  *  * * *
 * 
 * *  * * * and * * * 2 * and * * * 2 * * * * * * * and 2 and * * * 2
 * 4
 *  * * and * and * * * * * * * * * * * * * * * * 2 * 1 * * * * 2 * 2 * and * * * 2 * and * and * and and and and * * 2
 *  * to
 to
 * * * and
 * * 2
*2 * 2 * * 0 and * and * to 2 and *  to the * to * to the * to * to * to the * to * * to *  *  * 1 * * 2 * *
 * * * 0 * and the * * * * * 2 and and and and * and and and *
 in * * * 2
 *l and
* 2 * * 2
 * 2
 0
 * * 2
 * and * * and *f
 * *2
 and 2 * * * 2 * * * to * * * * *
 * and * and * and * * and * * and
 * 2
 * and * *  * * * * ( )

 * * * *  * * * 1
 * * * * * *  * * * * and
 * * * * * to  * * * * * * * * * to * * * and and * * * *  * * * *  * * * * * and * * * 2
 *  * *
  * 2
* * * *  * 9
* * 2
 and 2
 * 0 * and * and * * * * and * * 2
* * *
 and 2
 and 2
 *
*1
 * and * and * and
 1
 * 3
 * 2
*1 * 2
1 123
* 2 * * and * and and and and and * and and * and and ** * and and and
 and and 2
 * * 3 * 2
* * 2 * * * * * 2 * 2 * 1 and * and 2 * 2
* * * * *
* 1 * * * * 2 and 2
 * and the
 and 2 in 2 * * 2 * 2 * * * * 0 and 3
2 * 2 and 1 and and and and and 2 and and * 2
 2
* 2 * 2
*0
 *  * 2
 * and and * 0
* * * 2 * 2 and and and and 0 and and 23
 * and * and 0 * and 2 * 2 * * 2 2 * 2 1 * * 2 * 0 * *  * 3
 * * * * 2 * 2 * 0 0 1 2 * and * 2
* * * and * ** * and * and * * * * * * * 2
* * * * * and * 2 * * * * 2 * * * * * 2 *  * * * 2 * * * * * 2 * * *
* * * * * 2 and * and * and * * * *  * 2 * * *  * and * 2
 * * * * 9
 * 1
 and 2
*3
 * 1
 * * and * 2 * 2
 * 2 * to 2
 and  * 2 * * * * *  * 2 * * 2
 *  * * * * 2 * 2 * 2 * * to * 2  * 2 2 * * * 2 2 * * 2 * 2 * 2 2 * and 2 * * * * * * * 2 * * * * and 2 * 2 * 2 * * * 0
 * * * 2
 * * * 2
 2
 * 2
* 2
 *  * 2
 0
 * * * * * * * * * * *
 * * * 2
 *  * and 2
 * 1
* 2
* * * * * * * * * 
 and and * 2
 * 2
 * 2
 * * 2
 * * * * * * * * * * * 1
 * * 1 * * * * * * * * * and and and and * * * * * 2 * * * * * * and * * * * * * * * * * * * and * * * * * 2 * 0 * * * * * *
 * *  * and
 * * * * * * * * * *0

* * * 3 * and 4 * to 0 * 0 * 0.  * 2
 * * 0 * 2 * 1 * to 2 * 0
 *
 *
 * 1
* * * * 0 * 1 * 0 * * * * 2 *
 * * * 2
* * 0 * * * * * *2 * * * 2 * and 2 * * * * * *

 * *
 * * * * * * * * * and * * * * and  * * * * * * * * * * * * and 2 * and and and * * and  * and 23
 * 2
 *
 * 2
 *
 * * *
 * 2 * 3 *  * * 0 * * * * * * 2 * *
 * * * * *
 * and * and
 * 2 *
 * * * * * *  *

 and * * * * * * *
 * *2 * 2 * * * * * * * * * * *  * * * *  *  * *  * * * * * *  * 1 * 3 * * * * * * * * * * * * *  * * * * * * * * 2 2 * 2 * * * * * * * 2 * * * * * * and 0
 * * 2
 * and 3 * * * *  * * * *  * 5
 * * * and * and * and
* * 2 * 2 and and and * 1
 and 3
* * 2
 * for * 2 * 1
 * 1 * 2