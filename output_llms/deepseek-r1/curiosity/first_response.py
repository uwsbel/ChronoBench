import pychrono as chrono
import pychrono.irrlicht as irr
from pychrono import robot as cur
import math

# =============================================================================
# Custom Event Receiver for Keyboard Input
# =============================================================================

class KeyboardEventReceiver(irr.IEventReceiver):
    def __init__(self):
        super().__init__()
        self.key_pressed = {}
        self.steering = 0.0
        self.throttle = 0.0
        self.max_steer = math.pi/6  # 30 degrees max steering
        self.max_speed = 5.0        # rad/s wheel speed

    def OnEvent(self, event):
        if event.EventType == irr.EET_KEY_INPUT_EVENT:
            self.key_pressed[event.KeyInput.Key] = event.KeyInput.PressedDown
            return True
        return False

    def update_controls(self):
        self.steering = 0.0
        self.throttle = 0.0
        
        if self.key_pressed.get(irr.KEY_KEY_W, False):
            self.throttle += 1.0
        if self.key_pressed.get(irr.KEY_KEY_S, False):
            self.throttle -= 1.0
        if self.key_pressed.get(irr.KEY_KEY_A, False):
            self.steering += 1.0
        if self.key_pressed.get(irr.KEY_KEY_D, False):
            self.steering -= 1.0

        self.steering = chrono.ChClamp(self.steering * self.max_steer, -self.max_steer, self.max_steer)
        self.throttle = chrono.ChClamp(self.throttle * self.max_speed, -self.max_speed, self.max_speed)

# =============================================================================
# Main Simulation Setup
# =============================================================================

# Create Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetSolverMaxIterations(50)

# ------------------
# Terrain Setup
# ------------------
ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(50, 1, 50, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
system.Add(ground)

# ------------------
# Rover Setup
# ------------------
rover = cur.Curiosity()
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
rover.SetWheelVisualization(True)
system.Add(rover.GetChassis())

# Set wheel contact material
wheel_mat = chrono.ChMaterialSurfaceNSC()
wheel_mat.SetFriction(1.0)
for wheel in rover.GetWheels():
    wheel.GetBody().GetCollisionModel().SetAllShapesMaterial(wheel_mat)

# =============================================================================
# Visualization Setup
# =============================================================================

# Create Irrlicht application
app = irr.ChIrrApp(system, "Curiosity Rover Simulation", irr.dimension2du(1280, 720))

# Configure visualization
app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.AddTypicalSky()
app.AddTypicalCamera(irr.vector3df(3, 3, 3), irr.vector3df(0, 0, 0))
app.AddTypicalLights()

# Enable advanced visualization features
app.EnableShadows()
app.EnableContactDrawing(irr.E_CONTACT_DRAW_NONE)
app.EnableBodyFrameDrawing(False)
app.EnableCOGFrameDrawing(False)

# Add NASA logo to rover body
logo_tex = chrono.ChTexture()
logo_tex.SetTextureFilename(chrono.GetChronoDataFile('textures/nasa_logo.png'))
rover.GetChassis().AddAsset(logo_tex)

# =============================================================================
# Control System Setup
# =============================================================================

# Initialize event receiver
receiver = KeyboardEventReceiver()
app.SetUserEventReceiver(receiver)

# =============================================================================
# Simulation Loop
# =============================================================================

time_step = 0.001
sim_time = 0.0

app.AssetBindAll()
app.AssetUpdateAll()

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    
    # Update controls
    receiver.update_controls()
    
    # Apply controls to rover (differential steering)
    left_speed = receiver.throttle - receiver.steering
    right_speed = receiver.throttle + receiver.steering
    
    # Set wheel speeds (assuming 6 wheels: 0-2 left, 3-5 right)
    for i in [0, 1, 2]:
        rover.SetWheelMotorSpeed(i, left_speed)
    for i in [3, 4, 5]:
        rover.SetWheelMotorSpeed(i, right_speed)
    
    # Update camera to follow rover
    rover_pos = rover.GetChassis().GetPos()
    cam_pos = irr.vector3df(rover_pos.x + 3, rover_pos.y + 2, rover_pos.z + 3)
    app.GetActiveCamera().setPosition(cam_pos)
    app.GetActiveCamera().setTarget(irr.vector3df(rover_pos.x, rover_pos.y, rover_pos.z))
    
    # Simulation step
    app.DoStepDynamics(time_step)
    sim_time += time_step
    
    app.EndScene()