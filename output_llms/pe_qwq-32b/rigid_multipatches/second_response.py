hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))

def main():
    # ... code ...
    
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

main()

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Update simulation step sizes (moved before function definition)
    step_size = 2e-3
    tire_step_size = 1e-3

    # Create the HMMWV vehicle with updated initial position
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVectorD(6, -70, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create simplified terrain with single mesh-based patch
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # Create contact material with specified properties
    material = chrono.ChContactMaterialNSC()
    material.SetFriction(0.9)
    material.SetRestitution(0.01)
    material.SetCompliance(0.01)  # Set contact material thickness (compliance)

    # Add terrain patch using mesh file
    patch_coordsys = chrono.ChCoordsysd(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
    patch_file = veh.GetDataFile("terrain/meshes/Highway_col.obj")
    terrain.AddPatch(material, patch_coordsys, patch_file)

    # Add visual mesh to terrain ground body
    visual_mesh = chrono.ChTriangleMeshConnected()
    visual_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"))
    visual_shape = chrono.ChVisualShapeTriangleMesh(
        visual_mesh.GetVertices(), 
        visual_mesh.GetFaces()
    )
    visual_shape.SetFuzzyMaterials(True)
    terrain.GetGroundBody().AddVisualShape(visual_shape)

    terrain.Initialize()

    # Create visualization interface
    vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Setup driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

# Correct variable scope and data path setup
if __name__ == '__main__':
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    main()