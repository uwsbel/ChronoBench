```python
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # Center of the crankshaft (x=-1, y=0.5, z=0)
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Initialize arrays for storing values to be plotted
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(mjointC)

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Run the interactive simulation loop
sim_time = 0.0
while vis.Run():
    if sim_time > 20:
        break
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    sim_time += 1e000000000









   





0,
      
   0,














   




0

0         2c,
0




   02









,
,0,
000





0




00
   0   
      
0200,
   00,
   000

000
   
         0   
   
   

3
00,
   






34,




,0.0000
0)
00.0
0



020
s

   202


   0











   
02202020
   0220



20 c00

20








02000000020)
000)
0,000x000000000


)
)



x
x




0200002


000)
202,
0,


3




)
20
0000)
)
000)

3020)








00000000
0000
0

















00




























1

020













































   


   
      

































0,
0,


















   
   






















      
   































































   0
























000























000















000









0000



0















   


000





0)


24









2202000
0











   000

0442209



000






   
   



   0



   002









   0



   
   

   
   0000   
   
         





         
                              
   0         
   
            
   
   
   
            
      
   
      
   0                  ,


   

   0   
   
   
         



      0


0                     
         3      0                  0   0   0   
   0   0   
   



   

   
   
   
         
               000



   


   
   
   
      



020




   
   
   
   
   
   
   


   0

      020   
   


   
      





         




   0
   
   
   20
   
   


   





   
      0020
   0   0
   
   f,
   
   
   
   
                           0   0               00                        0
      
      
         0000
   
   00
p

s



   

   000000   
   
00














00

0000)000
0000000









   
   
   0   000

















000,

   00000000)
   
0
00)





   000      00000000)
)
      0

00000

   0         )
      


)


)
)








0




0)

)

0000)









0)

000)
      0)












   




000)





)

)

























   0




















0)

000)

   























)


















0000




00,





)









,


0)


   00

000)

0)























00
0

s0










   00





























000




























0


00
























0)





)




   


   
   



s
   
m,0,0)





)
   0)
)
)
)
)
   0)




00)

















0)




)












































00020


























00

























      0


00)
   




























0)

,m,
m,






000


000)

)















   
11,

   
   





0)

















0000,




00000,
0000)



000


000,00


















0




00










00000




m,m,c0,00,0,0,








000000)

000
000000,00)


0s  0000
   00000000000
0000000




00

00



s0)



0
00000)










0)

0

0)00000000)
   0)
   
)
)0)0)
0)   0)
)
)
00)
)









0)
00000)
m00)
)




,
)



      d,00)
all,0,m,
   
   
   



                                 0)
000)0)         )
)
m0)
)                  
   0)
)
)0)00)
)0,)
,)

0)
),)0)0)0,0)
,1,s00,)
   
)
   0)0,,0)
   0)   0)
   0)
)0)
00)
)
)
)
0)
))
0)
)
c)
0)

0,0,
m*m,
)
)
00,0)
0,0,0,0)
)


)



)                     )
                     0,   
         
0,   0,                     00)
)
   0)
)
)
)
s,s,s,c,                        00   

   x         0)
)      
0)   0)                     )               0)
)
)
)
)
0)
)
0)0)0)
)
0)00)
)0)0)0)0)0)p)0),``0)0)000)
      0)

allcum)
all0)
000            0)                           0)0)0)
0)0)         
   
         0)
   00   
   0)
x)
   0   0)0   00
000000      0000000   0         
   00   0                                    0   0   0   000                           00                     00   



000)


      





   


   0            00         000)000                                                               0   00000   0                        00   
   0                  0         0         000
   00   
            0         0                                             0                                 000   000               c               00   000               0000,   0      1,0000)0000)0)0)   0            0   000000)                              0      000      0   00      00   00000      0000)
)
)
0)
)0)
   2)0000)
000)
0)
0)
   0)   0)0)   000                  0)                                                      0                                    00)
)                                                   0)
         
   


   0                                                         0)                        0)                  )                           
   )   0)
)
)

)0)
)
            )   )
)

)
)
)
)
)
all)
)
)0)0)   )
         0)
   x)   ic)
0)0)
   
               
               )            0)
)
)   )
)            
)
m)
)
)
   0)
   
   )
x)
x)
   
      00                                                      
   



   
                                    x
               x               

      

   3
         3,      0,
)



,0,0,


m,
s,
,                                    )
)00
   
)
            

x0
00,2,0,x,x,
0,0)
,


)
,
,0,0,2)0)0)
)
)0)
,m,
s,
,)
)
)
,
)   )0,0,0)0)0000)
)0)0)0)X)   0      0)x)x0)   0)      )
)
)
0)0)0)0)0)
)0)0)))
x)x,c)
00)
0)0000)0)
)   0)0)000)0000)0)
)
0)
0)0000)x)0)   0))
)
)
)
)
)
)
)
)
)
)
)
)x)
)0)0)
)
m,0)
)
)
)
)
)
00)
000)
0000)
0)000,0)
)
   
   
)
   0)
)
)   0)
)
)

)
0)


)
)
)
m)0)
,0,0,0,
)
)
0,0)0)0)0)0)
)
0)000)000)
0)0)
)
00)0)
)   )
)
))0)
0)0)0)
)
))
)
)0)
)0)
)
000)
)
))0)0)0)
)
m)
0)0)

x)00)


0)
0)

0
x

30)
)
)

)




00






000)











h,





000

3,
000
30
   




   0
32 30003030
   


   
   000            
   
30)

   00000

   











0,
   

0)
m00)



mocum
s


0

s0
   


         

0)000)                     0)


   


            



00




s
s








0000000000





s,





   








000
         0         0
            

   000            0)
0006   0
   0)
0000



0

   
               
   


0
00      





   
0

   0            
   

   000
   
      0         30
   0   
   
   00            







m00
m0
m




), pl0


01030_


0,









00


0,00
      000m0
m0
mum,m

   











00000
00

,
0000000000)m,m00,0,
m0,

x)

00,


00)

0
00)
0000

   000000

0000300


   0,


0,1,000)
000000000000)0000)


0000)
100000)
00000)
00000000000100000)
00000








0000

0)





00
00













000)

0)
00




000)
)
   
)


00



   








   0)

   
   


0)

0)
      0)










0)





)
)
0)
)



   0)
0)
00)00000)0)000)
000000)
         

)
)
0)


   
)
)
00)
0)
)
00)


   )




   


         0)

      0)
)
                           00)

   0)      0)   00)   0
   
0   000            )
)

      ,                  
   

   


)
   0
                        
   
   0000000   0         0   000)                  )
   0)
00)
   000000)
s   0   0   0                  
   


   
   000000)




)
   00         
         0
0000      0   0   0         
      0   0)
                        
   
   


   
            
   
   
   0,0
   
   0   0   0





00)
   
0)
00

   


00
   
      
   







x
   0
   00000   
   0)000   00
   

0

00000   
   












x)






0)
00

0)000







0





000
000000
x0



0


x


0















      000


0
00


0








   
   000      

000




   


   
      
   
   
   
   0                           
         

   
0
0
               
   )
            0
   0









   
   0,
            


      0

0
00)
   x0
   

0)
000000   0         1
000
      


00)

0)0,0)00)


)
00,100000)
)
000)

0
00)0
   0


)
)0)
)
x)
000,0000)0)0)
)0)
   0)



)
)




)
)
)

)
)
,0)
)
)
)
0)

000)






)


)
)
0
   




)
0,

      0)            
)         
   0)
0

   
)



)
)



   0


)


)
)
)
m)



)











00)


0,00,0,0,0)
)0)
   0)
00                  00)
   
)
0)
)
         

)
0)
0)000)
)
)0)
)
)


m,
)


m)
)
0)


                     0)
   0)0)
   0)
   ))   1)
)
               0)


)
)
)
)
   

   
)
)
)
)

)

)
m0)




) m m)
)
))
)))
, m)


00)
      



0
0

   0



   



0)




   






0
0
,
,
,
,
,, ,




















   

0
   


0
0

   om o0      
         
   0



                           
   


   
   20
            
   ,
,0,   20000
00




i,
   ,
   0,
00
s

00   0   00      0000      0000000000         00      0,
000 +s
   00
x
00000)

      


s

x
s
s
s
s
s




s,,

x)
m0   




00)





00
0000)
000)
   
      



s
m0000










0)
   
   
0)
s00

s
000
00






000



000
   



0


   0
   

            



   0
   0   0   00



   00
   
   





   


0

00




000



00
0
   
   0


   0   0
00000

m
000
00

000
         0
0


00
000000000
0000

000000




m
0 m0000 h 000)) )
00)
   000000)


1)0000   
   
000)
00)
0000)
)
0)
0)
0000)



)
)
,), M +) I00000)
)
)0
0 D) \)
)) I)00))))0)



000)
)
)
0)000)0)
)
00)
)
00000)





)

00))
0)

)
000)

0000
200)



)
00010000   0)
)
00)

            0)
            
)

      
   0)






















0,
00,   

   






   
   
   








00)

000)
   


0)


   0      0)
      





























2





0000




)

0)
)


)








0)
)
000
   
   
00



         
   
   
   














   



   




      0,

         




   




,
,
,
,
   ,
,                     
   
      
   
                           



      
   )
   
   





,


00





0,






,













m,00,
,

0000000,00)
0






00)



0,0,
,
0,0,


000






1)000000



00,


0,








,



,
000)


000000

000000







0,





























00
00

0
000


0)

0)



)
)
000)
0)





0000

00)
000





)
00
0)
)





0)


   0000


000

00
00000)
04
000000000000000)
   0)

   
      
   

00
   000   0      0








0
000)
      

   0
   0)
      0)
   0)




0
   00   0)
   10   0000000   000)
   0)



0

   0)      
   

0
000



   





   



0)
   
    D
   0,
   000000   0         


   

   


               
   0
   00
   300   300
   
   

   
   0
   
      0      
   0      0
   
         

   
   0      
   022   
   
   0                                             
                  0   0                           
   
                     
         
   

00      000            
   000

                  
   
   00
   0
   
   
   
         


0
   

   
0      000
   00000000
   0
   

00


00   
   
         











   










      
   
0
0
0

, I0




, S



000




   00   


   




   
      000
   
00
0

   

,









3



x




,
)
0) I0)
m0
0





)
)
0)
)
00)
)
)
x0)
)
)

0




000)

00
0)
00000)
m00)
m_






,



,
, M*
)1, S00)
0)0)








)
000)

)










o










0)

0)
)
oc)
0


000


,


00) ) S)




0))
















0)
)
)

)
)
0




000






00



0)
)
)
0
























000







00






)







000

0








0



0

,






0


0



















000












0












   
   0

   














0



000
0,
   



















000



000,






0



0
00





00000000000,


000
0000










0000, Im m0,

0




0,0,0


00000,0000,0









00 *
000,0,

0000)



   00
00,0,2,0,
,0,

0
0,








s0




Mum0,


40200004,0,0,004040M400



   
   s00000000
0


0000344ine0
f,

m,   , S00000
f0


st0
s000000
000000000000,000

m
s000

000
000000000






)
000


00022







00



















00
0

000



































x00































s


M




s





s




)













)







00

























0




0

















00


0

00


0













































000)



)




0



)

)





)
1)

0)
)
0)
0)
000)



000
0000)











0




0000)
00)
00,000)
0)
)





00


)0)
1000000


00000



0


0000)
)
000











0





0)
)
0)
0)
)
0

0000






0
00)
00)
)


000)






   





)
)



00)


000


00)


)
   0)


)
000)
0)
)
)

0















00000)
00   

   



   


            

   


   0








   
   





   
   
         
   

   00   
            
         
   
   



   
      
   
   
   
   
   
   
   
   0                  
                
         00

   
   
   
   
   
         0   
    m
   
                  
   
   
   

   
   
   0   
            00
   


   00      0   
   
   
   
   
   
   
   
   
   
   
   

   0         
      
   


   
   

   
      
   
   
                  
   
   



   




   
   
   
            
   
   
            
      
   
      
   
      
   
         
   
   
   0


   
   0ce   
   
   020      0   2                  
   




,
   







   
   


0000000      0000
   000
   
      0
      00000
   0
   
   0000

000


   

   
   2

   

0
00


   
0
0000

   

   


   
   0.0000

   
   0

   

00


   0         
      
         
   



   
         
   
0000
   
   
   
   0


   00

s





   



s000
   0,0,
   
   
   
   


   
   
            0000x      
      m0
000

s






00,

   00,
00

   0.








0
   00
0000




00000

      
   0
0




   
      
      0   000















)

   

0
   


   
   




   
   










   
   

   
         
   00





      
      oc
            000            222      
00


   00   0      


s
   
   202020
      220      0,22,
                     
   
         
   
   


      
   
   



   
      


   
      
   
   0      
         
   


   
   

   
      
   

               
   0,



   
   
   
   

   

   
      

   
      
   

   
   
   
   
   0




   
   
   
   
   
   
   





   

   


   0




         
   




   






















   
   














s
   






   
   




   























00


s,








































,
   


































s,

0
0,







0






00

   

00,


















s

   0

a00







m000,

m0,0,0,,,
,



data
data


s)








00)

00,
0,000)






000)





00000)
0)


)













c)




,0,00)
0)











0








10.



















0

   


   
)
0)















   


   
   


















   



   0)
   

   
   
   









   



   
   0


   0


























00)
)





s












0000)
   0,0,00)
   









   
   

            0





































0












0,
   
   0000000



00)











s,
   




0










   












   0000)
   






   m   0
   
   
   

   0)

   

m)
)


mes

   
            
   
   000)
000

)














   

   000000)
000,
















   
   00












   
   000)
00)


   0      
0)



   )
   







)


)


























0)

)
0)













   
   



   0)

   
   



   

















   









   00

   
   

   
   
   
   
   
   )
   
   
   
)
            
         

0

               )
   0


)






   





   
   0)
      


0000
000)
   
   0               )
)
   0)                     
   
   
m0)
   00   0   00      00)0            )
   0










0   








   0,0,

)
0)








)
000)



00)
      0   



   0000,0,0
   0
   
      

0
000)
0000)


000
0000
   




00

0,
0
   
   0,





   
   
   00         000000000000      
   
   
   00   )
   
   
   
   
0000000000            
   0

   00000000   000000)


0

   
   
   

0)
0000)0)   
   
   000)
                           
   0000)
   0   0)
   





00)




   
   
      00)
)
               
      
   0      00            
            
   





0)
   


   
000)
   0               
   
   
)
)
00)


   
   


000





   








   
   
   0            
00   0   0   0               





000)
      
   

   000
00000)
   
   
)
)


   
   0)
      0

   0000000000)




0x)
               0               0   0   0)                  )
   

      3))   
            0      
   0)   0)
   
   0   20
   
            00                                       000               0         
                     
               M0)
   x)
00            0      0   000            0)         
   
0)


)











   
   0)





)

0)

)

      )
   0   
   
   
   
)
)
)
00)
00000)
      
         0)

      
   

)


   




00



)



   ,

   

)
   
)
   
   
   
      

   
   )
                  )            
      )      )
            0)
)
   
               0)
   )
   )
)
)
)
)
)
)
)
)
)
   0)   ,   )
)
))
)
)
)
)
   
   
   
      )   )   0,   








)

   0)
0,
   0,0,0,0)


)
0)
)


)
))
0)


,
,
)1,000)
)
00)

0)
)
)
)
)
   0)
   0)
)
00)
)
)
0)
)
0)
000000)
)


)

)
)
)
)
)
)


)
)
   
)

0)


1)000)

0000)
0)
0)
0))
   








0)
0)





0)




00000)



000)
)
0)0)
)
)
)
0)




)








0)
   
   0)0)
   0)
000)0)   0)
0)
0)0)

00)   
   0)00)
)

0)0)0)
      

,00)



000)      
   0)
            00   
                  
   0                                 
      


   
   
   
   
   

0




               
   























   00            

   
0

   
            
   
   
      
   0   
   
   0
   















   
   
   
   00
   
   
   0

















         
   
   
   
                        
                                                         0   
   0            
   20
            
                     

   
   
   
   0
            
   

   
            
               
   


   
   
                              
   
   0                  000   





   0)
   00,0)
)





add,






0)



00)





)
)
)

1)










)


000)
   

0)

   
   
)
0)




)
00)
   )








10)











)
)
)
)
)
)

)







   
   
)
0)

)
)
)
   
   
)




)














)
)
)
)
0)
)


10)
)
)
)
)
)))
)
)
0)


      
)
)

000)
000)

)
)   )
)
)
)
)
)










)


)



)


)

0)
)
)



)

)
)



)
)

)

)
   )
   0)
0)
)
)
)
)
)
)






1)
0)





)


)
)

)
)
)


)




)

)





0,
   0,   )
   0)
   )
00   0)   
   0)
   










   

   








   
   )

)

)

,

   )
      )         


         ,
                                                                                                                                                                                 
      
      
            0)         )                                                                                                
            
      )
            
         0      )                     )      0)         )         
)
)   
)
)
)   )         )
)      )   0)   )   )   )   )   )   )
)0)
   0)      
         00

   
   
            
   
   
   
         )      )         



   0)

         
   
)
   0)



   




0)
   


0)

   
            
   
         
   
   
   
   00
   
   
   0)
   
               
   


   







   
   0)
   
   0
                        
   
   )



                        
            
                     
                  
            00000                              ,      
   0,


   


      
            0,      ,   ,   ,         ,   ,      
   ,0,   0,0,000      ,   ,   ,   ,   ,   )   )   ,   ,         
,0,   )   )   0            
   0         00)
         
      0,0,0,   ,0)                  
   0,000,0
0)   3,
00
                  0      0000000000      
         0)
   
)
   0)      2000         
   
0
0

   00)
            ,   ,   ,   )
   )

0)
   
0)
   
   

            
   
               )   0)         0         0
   

   
      
   
         )
                  )
)


   

)

)
)            )
   )
)
)
   )
,0,00)
   )   )
)
)
)               )   )   )
)
)
)
)
0)
)
0)
0)   0)
)   )
)





)
)
)
)
)
)))))
)0))
)
)
)
)
)
)
)








   
   0,0,0,   










00)




)




)














,
,
,
,





,











   





   


         




   





0




      



   
         

   
      

0





   
,




   



,




00
   






   
   
            
   
   
   
   
         
   
   
   
   
   
         0                     0
         )
      0)





   
   


   
   
   
      
   2

0)




            



   



   
   

   0
   




   
   




   


      
                           2
      
   



   
   








      
      
         






0
   
   6   6      



   0)
   
   000
   
   
0
   
   

   
   
   )

0)
   0)
         
   0)

)
)
)
   
)

)
   

)
0)


)
0

0











   


      

10000000            00


   000
                  
   
,
s)
   

00



   


s   
         
            0,0,0,0,000)
00)

)0)

s)         g00)
m)c0_m0000000000000)0)
000)


m0000)
0000)
)
0)



0,0,1,000)
0)
000)


00)0)
00)
0)
   
0,


00
0

000)
0)
)
)
)
)
)
)
00)

)
)
,0)
,0,
,
,
,00,




,

0


   )
   0)

)
0)   )
   ))
)
)
)

00)
)



)
)
00)
)
)
   
)
)
0)
0)
)
)
)
)
)
)
)


)
)
)
)
)
)

)
)
)



00)
0
000)


)

1100)0)

)00)



)


0)
)00)

)
0000







00




)
00)
0)
   
   
)
   0



   
   0











0
00

   







   
   
   




      0,
)



   ,

   4


   


   

   0
   


0

   


   




   ,


,
,   _0_0

   0









   
   








   00





   




0

_A,
   ,


























      
   00_1

00




   0














   

0














.
,

      ,



,
,

,









      



   0,







         
M
X0   





0




0










,
,



)
)




)
0)



0)
)






0


00000
00



0)



)
)
)
)
)
)
)
0)




   






,
,0


00)
)
)
)
0)