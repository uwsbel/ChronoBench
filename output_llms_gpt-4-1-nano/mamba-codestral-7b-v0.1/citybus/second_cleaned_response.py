import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES
vis_type_mesh = veh.VisualizationType_MESH

chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-15.0, 10.0, 5.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.CityBus()
vehicle.Set_Contact_Method(contact_method)
vehicle.Set_Chassis_Collision_Type(chassis_collision_type)
vehicle.Set_Chassis_Fixed(False)
vehicle.Set_Init_Position(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Set_Tire_Type(tire_model)
vehicle.Set_Tire_Step_Size(tire_step_size)

vehicle.Initialize_()

vehicle.Set_Chassis_Visualization_Type(vis_type_mesh)
vehicle.Set_Suspension_Visualization_Type(vis_type)
vehicle.Set_Steering_Visualization_Type(vis_type)
vehicle.Set_Wheel_Visualization_Type(vis_type_mesh)
vehicle.Set_Tire_Visualization_Type(vis_type_mesh)

vehicle.Get_System().Set_Collision_System_Type(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.Set_Friction(0.9)
patch_mat.Set_Restitution(0.01)
terrain = veh.R
print("error happened with only start ```python")