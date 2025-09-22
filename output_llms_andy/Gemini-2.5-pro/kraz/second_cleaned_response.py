import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os




try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
except KeyError:
    print("Warning: CHRONO_DATA_DIR environment variable not set.")
    
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fallback_chrono_data_dir = os.path.join(script_dir, "..", "..", "chrono_data") 
    if not os.path.exists(fallback_chrono_data_dir):
        fallback_chrono_data_dir = os.path.join(script_dir, "chrono_data") 
    
    if os.path.exists(fallback_chrono_data_dir):
        print(f"Using fallback Chrono data path: {fallback_chrono_data_dir}")
        chrono.SetChronoDataPath(fallback_chrono_data_dir)
    else:
        print(f"Error: Chrono data directory not found at {fallback_chrono_data_dir} or via CHRONO_DATA_DIR.")
        print("Please ensure CHRONO_DATA_DIR is set or the fallback path is correct.")
        exit(1)

veh_data_path = os.path.join(chrono.GetChronoDataPath(), 'vehicle', '')
veh.SetDataPath(veh_data_path)




initLoc = chrono.ChVector3d(-15, 0, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  


vis_type = veh.Visualization.Type_MESH  


chassis_collision_type = veh.Collision.Type_NONE  


tire_model = veh.TireModel.Type_TMEASY  


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 200.0   



trackPoint = chrono.ChVector3d(3.0, 0.0, 2.1)  


contact_method = chrono.ChContactMethod_NSC



step_size = 1e-3
tire_step_size = step_size  


render_step_size = 1.0 / 50  


vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)  




if vehicle.GetSystem():
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
else:
    
    raise Exception("Vehicle system not found after Kraz() and SetContactMethod(). Cannot set collision system type.")

vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)  
vehicle.SetTireStepSize(tire_step_size)  
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)

vehicle.SetSuspensionVisualizationType(veh.Visualization.Type_PRIMITIVES)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())  

patch_coordsys = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT)
patch = terrain.AddPatch(patch_mat, patch_coordsys, terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Double Lane Change Demo')
vis.SetWindowSize(1280, 1024)

vis.SetChaseCamera(trackPoint, 25.0, 10.5)  
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()

vis.AttachVehicle(vehicle.GetVehicle())






dlc_steer_amplitude = 0.28   
print("error happened with only start ```python")