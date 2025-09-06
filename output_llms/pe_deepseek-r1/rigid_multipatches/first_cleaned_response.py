import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    
    
    
    
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
    
    
    
    
    
    hmmwv = veh.HMMWV_Full()
    
    
    init_pos = chrono.ChVector3d(0, 0, 0.5)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    
    
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    
    
    hmmwv.Initialize()
    
    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    
    
    
    
    terrain = veh.RigidTerrain(system)
    
    
    patch1_mat = chrono.ChContactMaterialSMC()
    patch1_mat.SetFriction(0.9)
    patch1_mat.SetRestitution(0.01)
    patch1 = terrain.AddPatch(patch1_mat, 
                             chrono.CSYSNORM, 
                             100, 50)  
    patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 50)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
    
    patch2_rot = chrono.QuatFromAngleY(0.2)  
    patch2_pos = chrono.ChVector3d(40, 0, 0.5)
    patch2 = terrain.AddPatch(patch1_mat, 
                             chrono.ChCoordsysd(patch2_pos, patch2_rot), 
                             30, 30)
    patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 30, 30)
    patch2.SetColor(chrono.ChColor(0.5, 0.4, 0.2))
    
    
    bump_mesh = chrono.ChTriangleMeshConnected()
    bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bump.obj"), True, True)
    bump_mat = chrono.ChContactMaterialSMC()
    bump_mat.SetFriction(0.7)
    
    bump_pos = chrono.ChVector3d(70, 0, 0)
    bump_rot = chrono.QUNIT
    bump_trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    bump_trimesh_shape.SetMesh(bump_mesh)
    bump_trimesh_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    
    bump_body = chrono.ChBody()
    bump_body.SetPos(bump_pos)
    bump_body.AddVisualShape(bump_trimesh_shape)
    bump_body.EnableCollision(True)
    bump_collision_model = bump_body.GetCollisionModel()
    bump_collision_model.ClearModel()
    bump_collision_model.AddTriangleMesh(bump_mat, bump_mesh, False, False)
    bump_collision_model.BuildModel()
    system.Add(bump_body)
    
    
    heightmap = veh.HeightmapTerrain(system, chrono.GetChronoDataFile("heightmaps/bump64.bmp"), 
                                    10, 10, 0, 2)  
    heightmap.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
    heightmap.Initialize(chrono.ChVector3d(100, 0, 0), chrono.CSD_ISO)
    
    
    terrain.Initialize()
    
    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV on Complex Terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, -10, 3), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVector3d(20, 20, 20), chrono.ChVector3d(0, 0, 0), 
                          50, 10, 40, 1024, chrono.ChColor(0.8, 0.8, 1.0))
    
    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    
    
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    
    
    driver.Initialize()
    
    
    
    
    
    step_size = 0.005
    realtime_step = True
    render_steps = 2  
    
    
    step_number = 0
    render_frame = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        
        driver.Synchronize(time)
        
        
        hmmwv.Synchronize(time, driver.GetInputs(), terrain)
        
        
        driver.Advance(step_size)
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1
        
        
        step_number += 1

if __name__ == "__main__":
    main()