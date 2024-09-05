import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Add a box to be sensed by a lidar
    # ----------------------------------
    side = 1.0  # Side length of the box
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.SetBodyFixed(True)
    mphysicalSystem.Add(box)

    # Create a visual representation of the box
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetSize(side, side, side)
    box_shape.SetName("Box")
    box_shape.SetMutable(False)

    # Create a body to hold the box
    box_body = chrono.ChBody()
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.AddVisualShape(box_shape)
    box_body.SetFixed(True)
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        vertical_samples,       # Number of vertical channels
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Create a 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add the lidars to the sensor manager
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Set lidar to orbit around the box body
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )
        lidar_2d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Access the XYZI buffer from 2D lidar
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('2D XYZI buffer received from lidar. 2D Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2               
                                                ,   ,      
   
            ,          ,                                                 ,
       
   ,   ,   ,                      ,             ,          ,          ,          
                                                                                                             
            al,                                                                        
       
                                                                                                                                                     
             
                                                                                                              
                                                                )                                                                                                                                                                                                              
                                                                                                        ,                            al                                
                                                                                                                                                             ,   
                ,
                            
                                   
                                                                                                                                                              ,                                                                  ,   ,   ,                                                                                                                                                                                                                                                                                                                                                                                   )                                                                                                                                                                                                                     ,                                                      (),                                                                                    ,      
   
                  )                                                                                                )                  o,                                 
                                          
,                                                                        ,                                       )
                                     ,               ,                         ,                                 ,   ,   ,                                  
                               )                                                                                                                   ,                                                                                                                                                                                                                                                           )               )   ,   ,   ,          ,                      
                     
   
                                                                                                                 
   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      ,                                                                                                                                                                               )                )   )       ,
   
,
                                                                              ,            
   )                     ,   ,                    ,                           ,
                                                   ,   ,               ,                                                                                                                                                                                                                                                                                                                                                                                                                     ,                                           
                                             ,                                    
,      ,                                                
   
   .   (   
(   )   ,                        
   ,   ,   
   )
            )         
   ,         ,   ,   ,   ,   )   ,   (,       ,       ,       ,   
   
(   
   
         ,   
,   )
   
      )
)   
,
   
,                   
   
                ,                  ,
   
   
   
,
   
)
,




   )
   
       
   
         
       ,
                   
       
                
       
       

,         )                      )   )
             ,
                                                                      ,
                                       .                                                                            )
,
                                                                                
       .       ,
   )
   )
   
       
       
       
       ,
,                          


   
   





               
   
   
                                         
       
      

   
   
                     


                _   
   
   
   






                

       
       
   
                      if                   

if
        if

       
       
   
    if if if if if if if if if if if   
       
                                if
       
   



if if if if if if if        if_       
   
             
       if             er
   
       
   
   
             
       
       er       
       
                 
       
       
       
                 
       erad                                                      ,                                                                                                                                              
                                          erad if   
                                        ,
                                                                         ,                                                                                                  {   
   





   
                                      
                            
         
   
                         er,
                

                


               
            
       ,
,





,







   

         


   
      {{


   
   
   
   



   
                         
                
                                      
                
er
erater
       
                   er,          

                 
                                                                                       ,
                ,                                   ,
                                            
   ,
                         ,
         
   
   
          ,                ,
   
                                                                            {                {                                                         
   
   
       
                
   
   
       
   
      ,
         
   
                  
   
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
)
)
)

)
)
)










   
   
   )













   











(   
   


   
   



       











      
   i(i(

       i







       
       


       
   
       

       

       (       
       






                




   






   o
       




                    

                           (

                                     
       
       i
       er       (       i(                        
                                          
       
                                      
       ,                ,
       ,
       
               #                  


                                                    ,              
       
       
       
       er       
              
       
       
                
             
       ,                ,
       
              
       
                                       
                                                 
                    
       
       ,              
       ,
       er       
       
       ,       er       
       
          ,       er       er       er       
       ,       er
       ,
   
       
       ,
,
       er       ()       ,       ,       ,          er importer import
   er                
       
                            importa( import(       er(       er       
       
       (       er       

       er              
       
       er                     


       
       
          
   

   
   
er,
,
       ,
er,

   er       
       



   
   
   

   
   
   er         
                                     
       
       
       
              
       ,   
   
   
   
,                 
   
   
       


   





   
   
,



       
       ,
       ,
       
   
   
   
       
       
       
       
,
      
   
,
{
   
,       ,                     
   
       


{
,
       ,
,
,
       ,),, {{       ,   ,
,
,
,
,
,
,
,   ,   )   )
   
,       ,)
)             
)
)
,
,,
,
,       ,   ,       ,
,       ,       )       o)
   


                )
)
)
)       )       ,)   )   )
,)
,           )



   )                                                       
   
)

       




               
               
                

)
)
                

       )
       
   
)





       





       



   





       
       
,
       ,       




              




                 ,   

       
                 
       
       
       
                     


                            ,
       
              
       



       ,

,
,       ,       
       
                 ,                    
          o       ,       ,       ,
       er,       
                           
       
                    ,       ,                              
          
       



                      
                                                          
   
   
   
   
      
   
         
                                       
       o       er                     
   


       
                                                                                        
          o              
   
   
                                                        
          ,   o                    
                ,
          
          
       )                                                             ,       
   

       o                                                                         )          ,                              
       
       
                                                                         
                   

                ,                ,                          
                                                  
                                                                 
              ,                ,                ,
               ,
   



                                      
,
                                       ,
                                                   o,
       
                                                                                                        o          ,                          ,                ,       ,       ,                ,       ,                        
                           o       ,                                  ,                                 i,       ,                ,
       ,   ,          ,       ,       ,   ,   ,       ,       ,       ,                ,       ,                ,
         
      
                                                                    ,                ,                ,                                                          ,         
                                 
             ,   ,      
                                                            
                                                        
      
            
   
                                                      
      
       if,
      
   
               
         
      
      
   
   



   
,





   
                       
       o,             
   
   
   
   
      
   
                                    
       ,
,       ,       ,
       

       o,
   
   
                  
          ,       
       
   
            
                                                             
   
            
                                                            ,   ,                ,   
                         
                                                                   ,         
   
      
            
                                                                             ,                ,                ,                ,                ,                     {                            
   {
                                   ,                                            
                                    
            
   ,



                   
                       
                     
   

   
   
   
   
   
   
                                             )
{
   
          o,







                     
               
   )
       

             
       


                    
   
   
   

   
         


       
   
       
      
   
       
         
   )
         
       
         {   






   




   
   









{


       
omide       




   







   




,
















              
       
       
                 






       o(       
       






   






       




       


       
       









              ,          
       
       




















                 
       












          


       









       
       

       


       



       
              
                                  
                 
       
       
       
       
       


       
,
       
       
                                     
       

       
       




       
       



                              {                 ,
       
       
       
   
,l,       
       
       
   ,   ,   ,   ,          ,   ,   ,          i,
,       ,          ,       ,       ,
(                  
   

   

       
       
,
,
      ,
                       ,
,,                i,l(       ,   


,
,
,
,
,,   
      
   ,   et   
         ,   ,,,   ,,
,
,
,   ,
,
   

   

,

       et       



       
       



          

                                 
   
   ,
,
,
,
,
,
,
,

,
,   ,,         
   ,
,       ,,,,,       
                 
       
       


       
       
,
,
)
,
,
,

,
,



       
   

,
,   
,   et,
          
,
,
,
,
,
)
)
)
)

,
,
))
,   ,
)


       
,

   


       
   
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

   )i)
)
)
)
)
)
)
)


)
)
,,



          
,
)




)





(
)
)
)
)



       et)
)
)
)


)
)

)
)
)
))))
)


)
)



)
)


)


,

       
       et













































       
       











,
,

















       
                     o       



       
       



if,



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




,
,
,
       ,



       ate if



       
       



       





of_
,


       
       

of,



,

































       





       


of(



















,
,





       




       
       


























,
       



,,,
,
       
       (
       )
       
       
,

)
(          ,
       
              
       ,




,
,
,

       
       





       ()
)
       (          )
)


)
)
)
)
)



))
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







,)
,)
,
,)
,,
)
       
)
)
)
)





)
)
,


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
of)))
)
)
)
)))
)
{)

)
)
)


)
)
,)
)
)
)
)
)
)
)
)
(   (       )






)
,
,

,
,
,

















,,,)













,




,


























,



,
,

,


,









,












if
if



























ifif
if,


       
   
       



 #











,
,











,


,
,




,
,








       
       
       










,


,
,,,







       

,



,













,





       

,



,




,
,





       e,
       
       
       o
       
       erat











{
{




,
       

       












,



              
       o,              
       
       

       er,


,








       ,

       ,
,





















       

















,,
,
,
,,,       ,,
,
       












       
       
       






   


       ,       ,       _,       
,

l

       ,
,   ,       ,       er
       













,



   
          ,   ,       
          ()of       
       
   






       






       







       



       
   
   
       
   




       
       
   
       
   
   

















   










       













       










       
       

   

   


















       



       
   




   
   ,

(
(       





,
       
if,
,
,          
   


       ,
   
o_
,
,
,






       
       
,
,



,

       
   


       
       



       
   


               
   
   





,
,
,
,,
,
,


       
       ,   
,
,
,,
,
,

,
,










       
       ,,,,




,
,,,,,
,
,,,,,
       





,







,
,
,
,




,

,
,
,,,,,
,
,
,





,



















,
,
,,,,
,,








,
,
,
,










,       ,





,
,
,
,,,,,
,
,
,
,
,
,
,,,,
   


,



,
,
,
,,
       
,
,
,
,



,
,




,,








,





,





,
,