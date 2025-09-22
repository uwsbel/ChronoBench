import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'chrono_data', 'vehicle', ''))









step_size = 1.0 / 50.0  


initLoc = chrono.ChVectorD(0, 0.6, 0)  
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  


tire_model = veh.TireModelType_TMEASY


camera_target_point = chrono.ChVectorD(0.0, 0.0, 1.75) 
camera_distance = 8.0  
camera_height_offset = 1.0 


terrain_height = 0.0
terrain_size_x = 200.0  
terrain_size_y = 200.0  





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) 



system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)





bus = veh.CityBus(system)


bus.Initialize(chrono.ChCoordsysD(initLoc, initRot))


bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH) 
bus.SetTireVisualizationType(veh.VisualizationType_MESH) 


bus.SetTireType(tire_model)




terrain = veh.ChRigidTerrain(system)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.9)
material.SetRestitution(0.01)
material.SetYoungModulus(2e7) 
material.SetPoissonRatio(0.3) 



patch_thickness = 0.2 
patch = terrain.AddPatch(material,
                         chrono.ChCoordsysD(chrono.ChVectorD(0, terrain_height - patch_thickness/2, 0), chrono.QUNIT), 
                         terrain_size_x, terrain_size_y, 
                         patch_thickness, 
                         True, 
                         2.0, 2.0, 
                         False, 
                         0.05) 



texture_file = veh.GetDataFile("terrain/textures/tile4.jpg") 
patch.SetTexture(texture_file, (terrain_size_x / 10.0), (terrain_size_y / 10.0)) 


patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))


terrain.Initialize()







app = veh.ChVehicleIrrApp(bus, "CityBus on Rigid Terrain Simulation", chrono.dimension2du(1280, 720))



app.SetChaseCamera(camera_target_point, camera_distance, camera_height_offset)
app.SetTimestep(step_size) 


app.AssetBindAll()
app.AssetUpdateAll()




irrlicht_path = os.environ.get('CHRONO_IRRLICHT_DATA_DIR', '../../../chrono_data/irrlicht/')
app.AddTypicalSky(irrlicht_path + "skybox/")



app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)






driver = veh.ChInteractiveDriverIRR(app)


driver.SetSteeringDelta(0.04)  
driver.SetThrottleDelta(0.04)  
driver.SetBrakingDelta(0.1)    
driver.SetGains(2.0, 5.0, 5.0) 


driver.Initialize()








time = 0.0

print("\nPyChrono simulation started. Controls:")
print("Steering:  A/D or Left/Right Arrow Keys")
print("Throttle:  W or Up Arrow Key")
print("Braking:   S or Down Arrow Key")
print("Camera:    Mouse or C (cycle modes), P (projection), T (tracking)")
print("           PageUp/PageDown (zoom), NumPad Arrows (pan/rotate)")
print("Quit:      ESC key\n")


while app.GetDevice().run():
    time = system.GetChTime()

    
    app.BeginScene(True, True, chrono.ChColorToIrrColor(chrono.ChColor(0.3, 0.3, 0.4))) 
    app.DrawAll() 

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) 

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    bus.Advance(step_size)
    app.Advance(step_size) 

    
    app.EndScene()

    
    
    
    
    
    



print("Simulation ended.")