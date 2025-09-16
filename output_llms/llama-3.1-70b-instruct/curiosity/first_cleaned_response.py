import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine
import numpy as np


mysystem = chrono.ChSystemNSC()
mysystem.SetDt(0.01)


ground = chrono.ChBodyEasyBox(mysystem, 10, 2, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -3, 0))
ground.SetBodyFixed(True)
mysystem.Add(ground)


rover = chrono.ChBodyEasyBox(mysystem, 2, 1, 2, 100, True, True)
rover.SetPos(chrono.ChVectorD(0, 1, 0))
mysystem.Add(rover)


motor_driver = chrono.ChMotorLinearSpeed()
motor_driver.SetSpeed(chrono.ChVectorD(0, 0, 1))
rover.AddDriver(motor_driver)


viz = chronoirr.ChVisualSystemIrrlicht()
viz.SetWindowSize(1024, 768)
viz.SetWindowTitle("Curiosity Rover Simulation")
viz.SetCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))
viz.SetLightDirection(chrono.ChVectorD(1, 1, 1))
viz.SetShowAxes(True)
viz.SetShowLogo(True)
viz.SetSkyBox(True)
viz.SetShadows(True)
viz.SetTextures(True)


rover_texture = chrono.ChTexture()
rover_texture.SetTextureFilename("rover_texture.png")
rover.AddVisualizationAsset(rover_texture)


ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename("ground_texture.png")
ground.AddVisualizationAsset(ground_texture)


logo = chrono.ChVisualizationLogo()
logo.SetLogoFilename("logo.png")
logo.SetLogoPosition(chrono.ChVectorD(0, 0, 0))
logo.SetLogoScale(chrono.ChVectorD(1, 1, 1))
viz.AddVisualizationAsset(logo)


def collision_callback(contact_info):
    print("Contact detected between rover and ground")

mysystem.SetContactCallback(collision_callback)


while viz.Run():
    mysystem.DoStepDynamics(0.01)
    viz.BeginScene()
    viz.DrawAll()
    viz.EndScene()