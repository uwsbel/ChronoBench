import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager and configure lidar sensor
sensor_manager = sens.SensorManagerPtr()
sensor_manager = sens.SensorManager()
sensor_manager.AddSensor(sens.LidarSensor())
lidar_sensor = sensor_manager.GetSensor(0)
lidar_sensor.SetMaxRange(10)
lidar_sensor.SetMaxMeasurements(100)
lidar_sensor.SetMeasurementFrequency(10)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Median)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_2)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_3)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_4)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_5)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_6)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_7)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_8)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_9)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_10)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_11)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_12)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_13)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_14)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_15)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_16)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_17)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_18)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_19)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_20)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_21)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_22)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_23)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_24)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_25)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_26)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_27)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_28)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_29)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_30)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_31)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_32)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_33)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_34)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_35)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_36)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_37)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_38)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_39)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_40)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_41)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_42)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_43)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_44)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_45)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_46)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_47)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_48)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_49)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_50)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_51)
lidar_sensor.SetMeasurementFilter(sens.LidarSensor.Filter_Kalman_Smoothing_Saturated_5






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


)



))
)


))
)
)




)
)
)
)))
)))
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
)




)
)




























)











44




































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
)))
)
)
)))
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
)

)
)
)
)
)
)
)
)))))))))))
)))))
)))
)
)))
)
)
))))
))
))))))))
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
)
)
))
)
))
)
)
))
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
)))
)))
)
)
))))
)
))))
)
)
)
)
)))))
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
)))))))
))))))))))
))))
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
)
)
)
)
)
)
)
)
)))))))))))))))
))
))
)))
)
)
)
)
)
)
)
)
)))
)
)))))
)
)
)
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
))))
)))))))))))))
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
))))
)
)
)
)
))))
)
))))
)
)
)
)
))))
)
)))))
)
)))
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
))


))
)
)
)
)
)
)))))))))
)
)))))))
))))))))
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
))
)
.
.)
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
)
)
)












)
))
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
))
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
))
)
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

)
)))





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
))))

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























)




)
)

r

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






)))
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
)
)
)
))
)
))
)
))))
)
)
)
))
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
)
))
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
))))))
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
)
)))
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
))))))))
)
))
)
))
))
)
))
)
)
)
)


)

)
))))))
)
))))
))))))))))))))))))))
))))
))))
)

)
)
)

)
)
))))))))))))))))))))
)
))
)))))
)
)
))
)
)
))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))
))))))))))))))))))))))))))))))))))))))))))))))
)))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))
))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))
)
))))
)
))))))
)))))))))))))))))))
))))
)
))))))))))))
))
)
)
))))
)))))))))))))))))))))))))))))))))))))))
))))))))))))))))))
)
))))))))))))))
)
)
)))))
)
)))
)
)
)))
)
))
))))))))))))))))))))))
))))))))))))))))))))))))
))))))))))))))))))))))))))))))))))
)))))
)
))))
)
)))))))
)))
))))))
))
)
))
))
)
))))))))
)
)
)))
)))))))))))))))))))))))))))))))))))))
)
)

)
)
)
)
)
)))
))))
)
)))))))))))))))))))))
)
))))
)
))))))))
)
)
)
))
)))))))))))))
)
)))
)))))))))))))
))
)
))))))))))))))))))))))))))))))))))