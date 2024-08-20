import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np
import pychrono.sensor as sensor


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 20  


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


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   
                            0,     
                            1.1,   
                            0,     
                            30,    
                            0.01,  
                            2e8,   
                            3e4    
)


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),40, 40, -1,1, 0.02)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()


sensor_manager = sensor.ChSensorManager()


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