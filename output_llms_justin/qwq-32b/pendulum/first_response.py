import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Create the Chrono system with gravity
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))

# Collision and visualization for ground (a flat box)
ground.GetCollisionModel().AddBox(1, 0.05, 1)  # Half extents: x=1, y=0.05, z=1
ground.GetCollisionModel().BuildModel()
ground.AddAsset(chrono.ChVisualShape(chrono.ChBox(chrono.ChVectorD(1, 0.05, 1)),
                                    False, chrono.ChColorAsset(0.3, 0.3, 0.3)))

# Create pendulum body
pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))  # Center of cylinder

mass = 1.0
pendulum.SetMass(mass)

# Compute inertia for cylinder (radius=0.1, length=1.0 along Y-axis)
radius = 0.1
length = 1.0
inertia = chrono.ChVectorD()
chrono.ChBodyAuxiliary.CylinderInertia(mass, radius, length, chrono.VECT_Y, inertia)
pendulum.SetInertiaXX(inertia)

# Collision and visualization for pendulum (cylinder along Y-axis)
pendulum.GetCollisionModel().AddCylinder(radius, length, chrono.VECT_Y)
pendulum.GetCollisionModel().BuildModel()
pendulum.AddAsset(chrono.ChVisualShape(chrono.ChCylinder(radius, length),
                                      False, chrono.ChColorAsset(0.8, 0.2, 0.2)))

# Create revolute joint between ground and pendulum
joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
joint.SetAxis(chrono.ChVectorD(1, 0, 0))  # Rotation axis along global X
system.AddLink(joint)

# Add bodies to the system
system.Add(ground)
system.Add(pendulum)

# Initialize Irrlicht visualization
app = chronoirr.ChIrrApp(system, 'Pendulum Simulation', chronoirr.dimension2du(1024, 768))
app.AddAllTypicalLights()
app.AddTypicalCamera(chronoirr.ChVectorD(2, 2, 2), chronoirr.ChVectorD(0, 0, 0))
app.AssetBind()
app.AssetUpdate()
app.SetTimestep(0.005)

# Simulation loop
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStepDynamics()
    
    current_time = system.GetChTime()
    if current_time % 0.1 < app.GetSystem().GetStep():
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.2f}, Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
    
    app.EndScene()