import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np
import pychrono.sensor as sensor

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 1.2)
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
render_step_size = 1.0 / 20  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
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
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),40, 40, -1,1, 0.02)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

# Create the sensor manager
sensor_manager = sensor.ChSensorManager()

# Add a lidar sensor to the vehicle
lidar = sensor.ChLidar(sensor_manager, vehicle.GetSystem())
lidar.SetMaxRange(10.0)
lidar.SetMaxMeasurements(100)
lidar.SetMeasurementFrequency(10)
lidar.SetMeasurementRange(10.0)
lidar.SetMeasurementResolution(0.1)
lidar.SetMeasurementFilter(sensor.ChLidar.Filter_Gaussian)
lidar.SetMeasurementFilterSigma(0.5)
lidar.SetMeasurementFilterBeta(0.0)
lidar.SetMeasurementFilterAlpha(0.0)
lidar.SetMeasurementFilterAlpha2(0.0)
lidar.SetMeasurementFilterAlpha3(0.0)
lidar.SetMeasurementFilterAlpha4(0.0)
lidar.SetMeasurementFilterAlpha5(0.0)
lidar.SetMeasurementFilterAlpha6(0.0)
lidar.SetMeasurementFilterAlpha7(0.0)
lidar.SetMeasurementFilterAlpha8(0.0)
lidar.SetMeasurementFilterAlpha9(0.0)
lidar.SetMeasurementFilterAlpha10(0.0)
lidar.SetMeasurementFilterAlpha11(0.0)
lidar.SetMeasurementFilterAlpha12(0.0)
lidar.SetMeasurementFilterAlpha13(0.0)
lidar.SetMeasurementFilterAlpha14(0.0)
lidar.SetMeasurementFilterAlpha15(0.0)
lidar.SetMeasurementFilterAlpha16(0.0)
lidar.SetMeasurementFilterAlpha17(0.0)
lidar.SetMeasurementFilterAlpha18(0.0)
lidar.SetMeasurementFilterAlpha19(0.0)
lidar.SetMeasurementFilterAlpha20(0.0)
lidar.SetMeasurementFilterAlpha21(0.0)
lidar.SetMeasurementFilterAlpha22(0.0)
lidar.SetMeasurementFilterAlpha23(0.0)
lidar.SetMeasurementFilterAlpha24(0.0)
lidar.SetMeasurementFilterAlpha25(0.0)
lidar.SetMeasurementFilterAlpha26(0.0)
lidar.SetMeasurementFilterAlpha27(0.0)
lidar.SetMeasurementFilterAlpha28(0.0)
lidar.SetMeasurementFilterAlpha29(0.0)
lidar.SetMeasurementFilterAlpha30(0.



)




























)









)
)
)
)





)






)

















)
)


)






)
)




)

)















)



)









)
)

)
)
)




)
)
)
)
)
)
)
)





)








)



)








)
)


















































































)





)
























)
















)
)













































)
)



















)








)


)
)
)
)
)










)





)


















)
)































)
)


)







)




)








)
)
)
)
)


)
)
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





)
)


)

























)



)
)




















)

















)
)
)





)
)








)













































)














)
)
)












)
)











































)
)
)
















)





)





)

















)





)
)

)
)
)
)









)
)









)




)
)


)
)
)
)
)
)









)






























)








)






)
)


)































































)













)

)
)












)














)
)







)




)
)
)







)





















)




)
)
)
)
)

)






)
)
















































































































































































)
)




)






































)



)




















)





)
)
)









































)
)








)
)
)

)
)









)
)




)
)








)









































)






















































































































































































































































































































































































































































































































































































































































































































































































































































































































































)






















































































)



)
)
)













)
)























)
)
)
)
)



)
)
)




)


)



)
)
)










)


)
)
)


)
)



)
)










































































































































)
)
















)










)
)
)
)














)

)
)

)
)


)
)









)
)
)




)






















)
)
)
)









)

















)
)











)
)






)































)
























)
)








































































)
)

















)































)












































)
)
)


)





)


)
)





)
)
)
)
)
)













)






)


















)
)
)








)






























)
)



)
)
)


)

)









)









)
)
)
)
)
)
)
)




)
)




















)
)















)

























)
)

)
)






















)






















































































)
)




)















































)
)








)



















)
)















































)
)










)










)




)


)




)
)





)









)










)
)
)





)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)

)
)
)
)
)
)
)
)
)
)
)
)












)
)
)




)
)



)
)
)





)
)


)
)




)
)
)
)
)
)
)
)
)

)
)


)
)
)













)

)



)
)
)
)

)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)





)
)
)


)
)
)










)













)
)
)




)

)
)
)
)
)
)
)
)
)
)
)







)




)
)













)
)
)






)
)


)
)
)
)






)
)
)
)
)
)
)
)
)


)
)








)








)

)
)
)
)
)
)
)
)
)
)
)
)
)
)
)

)
)

)
)



)
)
)
)
)
)

)
)


)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)











)
)
)
)
)
)

)





)
)
)
)
)
)
)









)

)
)
)







)
)





)
)
)
)
)
)





)
)
)















)
)


)

)
)
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
)
)
)

)
)
)
)
)
)
)
)
)
)
)
)
)
)



)
)
)
)
)



)
)
)
)
)
)
)
)
)


)






)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)












)
)
)
)
)
)


)
)
)

)


)
)
)
)


)
)






)
)
)
)
)
)

)
)
)
)
)
)
)






)
)
)
)








)














)
)
































)
)

)
)










)





















)


)
)
)
)






)
)




)
)
)
)
)
)
)




)
)
)
)
)
)
)



)





)
)
)








)
)
)
)
)
)
)
)
)


)



)
)
)
)
)
)
)
)


)
)
)
)
)
)
)
)



















)
)
)
)
)
)
)
)
)
)
)
)












)
)


)





)
)

)
)
)
)








)