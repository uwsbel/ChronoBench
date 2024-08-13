import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()  # veh.HMMWV_Reduced() could be another choice here
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the SCM deformable terrain patch
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                            0,     # Bekker Kc
                            1.1,   # Bekker n exponent
                            0,     # Mohr cohesive limit (Pa)
                            30,    # Mohr friction limit (degrees)
                            0.01,  # Janosi shear coefficient (m)
                            2e8,   # Elastic stiffness (Pa/m), before plastic yield
                            3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# -----------------------
# Create a sensor manager
# -----------------------
manager = sens.ChSensorManager(vehicle.GetSystem())
intensity = 0.1
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3    , 


  0,  
          0  
      
  
    





    




        








  




          

      00  
      



    

  
      


          


      0  
        



    



  
                


        
                                                                                                                                                                                 
                                                  
                                                          
  


                                  
                                                                                  00                                              00                                000000)                                                                        0        
                                        



        


                                                                                                    
  


                                                                            
  


          


        0    
  
0                                    





  4




000      





  
    











000,







000



  













































000
























000


0












,







)
)
)









































0




,








)
























)























,
,
,
,



,  
    

0,0,

,
0)





)
)























00











      















0

,


  



























































    










    




  2  




















, 0




























   #  

            
  



      
  


  
  
              000          0    




       




   #



  


            
0  

    

      



        0


  
      0  










  



    




00





0
00


  































































    
  









00







000      
              ,  














o







o     
  

,  4,  









  00000o

    0  





o,
,





f #




,
,






0








































,






0
00




0























0


















































00







0





 
   #












0 2  


0















,
, 


, #









,


,











0,






,


,
,
,

,  

,







00,
,







,



,

  
, 
, 
,
,
,  



,
,
,

,
,


0,


      ,  ,





,








,





,





,






     
 
 
    


         
 
  00,
,
,
)
)



         

          
,


)
)

2    




   
  



,

        

      
  
                        ))
,
,  )
,,,


          



  



)





0 







,      
     

,
  
   
,


,  
  
  
   
   f     00  
)
)
  







  
 
  

  
  
  ,







)





0

  



0  


      0    




00)






0  



000  




000






















00







0

  


0000  0  


00


    
0000  



2)        

00  










      




    

















    



  
  
  
  
 

    #



   # #  




                      0)


























00)

)  






,
,




    













 







0f






      





00

      








,  ,  

  0        
















      0              


      


                        
      




  




    


,
,  )
)
,


)
)
    




,  
  
,



























)
)






,  





























)
)




  0)
)

)



























  )



   #,

h,  


  
      
  
      



  
)




      


  
  






























)
  )

















  











)









)
)





  )

)
)  )
)
)










)
)
)
)
)
)



  )  0)

)
  )
)  )  )  )
)      )    )  )
)
)
)  h)
)
)



)
)

000)
  
  
)
)
)
)
)
















0
0




0)











)



)





















































































00






























0


















































































































































































































































































































































)


























































000)
0)






















00000)

0)





000








00)





















  
00)
)
)  )  )
)


))





)



)






)


00)





,

00



0)

































  





0)


















)
)

0)






























00  





000,  000)
)



)
)






)
00)

)
)
)

,  0)
00,  0)
)
0,
,000)




00)






)
0  0,    
00,  ,  0,
0)
)
      )
)  0  

)
)
)
)
)





,

0)


)
)
)
0000)







00
0,
,





,





,  0,


000,

0  


)
00       import  000000)

0)

000000ree
0000

000,


00000000




00





0000,  0000  0



um00ree,



0)
0000.00





000000  0ree

00000
000000














00


   0000   #  
00000  



0  0)



00  




00000)
000)


0000)
0000)


00000  



00)



ide)








  0  
0000  
    
000)



                000)



0  00000





0




0  

0)












00




000



  


      0  0  00  


0  00
0,       ,
          

      0,
000



,






,




  
  


  0  


  
  0  0                                                                        
     0                    0  0)
 import,  0es,   import  0       0    00   import   import,  


  000ree,  import,00   import,  )
00  0              000  
)
)






00  




0









000



)






0

0  


00000













0
















   






0reance,


00   

  

























   
0000       .


    ,  ,     
,
.



      0
















0,   import  0,  .      .
  
.





)


















.











.
.



  )      


  


     







yess















    











  .   



  










   
   












00
























  0






h






000












0


















,





,


















,



















,



,























































,














00





00





   










0





   
   



000


0
0




  




0
h0


00




   






























00
0



















































































0











0






   
   





















.



















0




   0   





00
















022.



0










   
00.


































000




0

























000





























0
0





















0000
































































0







































































.



















,
,
,




,,,,

      
   0.














.
.







  

   0.0.


02.


00.

.















,
,



,













)
)
,
,
,

)




)
)






0







)
)




)
)
)

)))
)





)


)





)

)))))
)

00)
)
0000)
)
)))
)





   
   00  
   
   

   
   ,

        es00.0  00)  ))




0,
,

int00)



  
  00.

    import.    importess,000000)
)
000)
)
0)
   000.       )
   0,


0,
2.
to.
00)
)

000)
)
0)
)
00)


)
)
,))
0)  )
000000)
0,00


00.0.
h00000,0000,
0,00,
   0)
00000)
))))000)
),
00000000)
),
h.
,
0),),)
00000000)

0,
0)

,
)
000000)
))



for,



00 for0)00000.
)


)
re)
)

000)
)
)



)




)






)
)
0)
0000,















0  






0)
for000)

   


10,
int,


000)










00

00000


000000)
0)



def000)






0,


int)
000000)
00)






)













000)
)




























0
















00000


10














000



)









00



)
   















   





  00   00     0  000)
       )
                                00,    #

         0   ,   0)
   
)
   00   


   0        

            
                                                  
         000   



                  
         
   








   

   
   
   


   






   


,
,












   
   
   
   




            
   

   
   )   1)
)
















,
,


)

)

      

   0,
)
)





















   
   00   




 import import   


           

)











00)
)









  











00   


)






           

   

      

0


   
000)

)
0)




  







   








0)





   


        
)
)

0)
)

00,

00)
)
      0)
      


   0      
00)
)
)
0,

)000)    import)

         )