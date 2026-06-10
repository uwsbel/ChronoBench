import os
import struct
import tempfile
import math

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")







def create_gradability_heightmap_bmp(filename, nx=256, ny=256):
    
    row_stride = (3 * nx + 3) & ~3
    image_size = row_stride * ny
    file_size = 54 + image_size

    with open(filename, "wb") as f:
        
        f.write(struct.pack("<2sIHHI", b"BM", file_size, 0, 0, 54))

        
        f.write(
            struct.pack(
                "<IIIHHIIIIII",
                40,          
                nx,
                ny,
                1,           
                24,          
                0,           
                image_size,
                2835,
                2835,
                0,
                0,
            )
        )

        
        for j in range(ny - 1, -1, -1):
            row = bytearray()

            y = j / float(ny - 1)

            for i in range(nx):
                x = i / float(nx - 1)

                
                
                hill = 0.78 * max(0.0, 1.0 - abs(2.0 * x - 1.0))

                
                bump = 0.22 * math.exp(
                    -(
                        ((x - 0.72) ** 2) / (2.0 * 0.045 ** 2)
                        + ((y - 0.50) ** 2) / (2.0 * 0.120 ** 2)
                    )
                )

                h = max(0.0, min(1.0, hill + bump))
                gray = int(255 * h)

                
                row.extend([gray, gray, gray])

            
            while len(row) < row_stride:
                row.append(0)

            f.write(row)







initLoc = chrono.ChVector3d(-45.0, 0.0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0.0
terrainLength = 100.0
terrainWidth = 100.0


num_patches = 4
patchLength = terrainLength / num_patches


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50.0






vehicle = veh.Gator()
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








patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())


heightmap_file = os.path.join(tempfile.gettempdir(), "chrono_gradability_heightmap.bmp")
create_gradability_heightmap_bmp(heightmap_file)


x0 = -terrainLength / 2.0 + patchLength / 2.0

patch_centers = [
    x0 + 0 * patchLength,
    x0 + 1 * patchLength,
    x0 + 2 * patchLength,
    x0 + 3 * patchLength,
]


patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(patch_centers[0], 0.0, terrainHeight),
        chrono.QUNIT,
    ),
    patchLength,
    terrainWidth,
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 60, 60)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(patch_centers[1], 0.0, terrainHeight),
        chrono.QUNIT,
    ),
    patchLength,
    terrainWidth,
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 60, 60)
patch2.SetColor(chrono.ChColor(0.4, 0.8, 0.4))



patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(patch_centers[2], 0.0, terrainHeight),
        chrono.QUNIT,
    ),
    heightmap_file,
    patchLength,
    terrainWidth,
    0.0,
    2.0,
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 60, 60)
patch3.SetColor(chrono.ChColor(0.55, 0.45, 0.30))


patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(patch_centers[3], 0.0, terrainHeight),
        chrono.QUNIT,
    ),
    patchLength,
    terrainWidth,
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 60, 60)
patch4.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

terrain.Initialize()







def add_speed_bump(system, material, x, y=0.0, z_base=0.0):
    bump_x_size = 0.65
    bump_y_size = 6.0
    bump_z_size = 0.20

    bump = chrono.ChBodyEasyBox(
        bump_x_size,
        bump_y_size,
        bump_z_size,
        1000.0,
        True,
        True,
        material,
    )

    bump.SetName("terrain_speed_bump")
    bump.SetFixed(True)
    bump.SetPos(chrono.ChVector3d(x, y, z_base + bump_z_size / 2.0))

    system.Add(bump)
    return bump


add_speed_bump(vehicle.GetSystem(), patch_mat, patch_centers[0])
add_speed_bump(vehicle.GetSystem(), patch_mat, patch_centers[1])
add_speed_bump(vehicle.GetSystem(), patch_mat, patch_centers[3])






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator vehicle - four terrain patches with bumps")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())






driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    
    realtime_timer.Spin(step_size)