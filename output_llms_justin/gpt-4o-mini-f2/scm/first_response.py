import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid body tire data
tire_data = veh.RigidTireContactData()
tire_data.mass = 50
tire_data.inertia = chrono.ChVector3d(2.5, 25, 25)
tire_data.radius = 0.5
tire_data.width = 0.4
tire_data.point_contact = True

# Rigid tire model
tire_left_f = veh.RigidTireContact(tire_data)
tire_right_f = veh.RigidTireContact(tire_data)
tire_left_r = veh.RigidTireContact(tire_data)
tire_right_r = veh.RigidTireContact(tire_data)

# Chassis tracked by the camera
track_chassis = veh.TrackedCamera_CHASSIS

# Type of terrain for vehicle to interact with
# terrain = veh.TerrainModel_SCMD()
terrain = veh.TerrainModel_FLOOR()

# Poon chassis tracked by the camera
track_chassis = veh.TrackedCamera_CHASSIS

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize

vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced()  could be another choice here
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
terrain.Initialize(vehicle.GetSystem())
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)
terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.AddPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, "Sinkage")
terrain.SetMovingPatch(10, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), True)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh.GetDataFile("terrain/textures/bump64_x64.jpg"), 6.4, 6.4)

terrain.SetSoilParameters(0.2e6,   # Bekker Kphi
                            0,       # Bekker Kc
                            1.1,     # Bekker n exponent
                            0,       # Mohr cohesive limit (Pa)
                            30,      # Mohr friction limit (degrees)
                            1.0,     # Janosi shear coefficient (m)
                            2e8,     # Elastic stiffness (Pa/m), before plastic yield
                            3e4      # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain.Initialize(veh.GetSystem())

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetMeshName(veh.GetDataFile("terrain/meshes/bump64_x64.obj"))
terrain.SetTexture(veh
print("error happened with only start ```python")