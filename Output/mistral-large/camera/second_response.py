# ... (previous code)

# Define the camera offset pose relative to the body it is attached to
offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 2), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

# ... (previous code)

# Add point lights to the scene for illumination
intensity = 1.0  # Set the light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
# manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
# manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
# manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

# ... (previous code)

# Camera orbit rate
orbit_rate = 0.1   # Rate of the camera orbit in radians per second

# ... (previous code)

# Image width and height
image_width = 960
image_height = 480

# ... (previous code)

# Save camera images
save = True

# ... (previous code)

# Remove unused lens model
# lens_model = sens.PINHOLE  # Pinhole lens model

# ... (previous code)