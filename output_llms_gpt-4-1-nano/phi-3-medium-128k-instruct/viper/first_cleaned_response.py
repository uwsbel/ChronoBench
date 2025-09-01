import chrono

import irrlicht

import numpy as np




chrono.Init()




system = chrono.ChSystemNSC()




system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground_shape = chrono.ChBodyDSG(chrono.ChEntityExt::CreateBox(system, 10, 1, 1))

ground_shape.SetPos(chrono.ChVectorD(0, -0.5, 0))

system.AddBody(ground_shape)




viper_rover = chrono.ChViperRover(system)

driver = chrono.ChDriverSeparate(system)

driver.Set_Enabled(True)




viper_rover.SetPos(chrono.ChVectorD(0, 0, 0))

viper_rover.SetEulerRPY(0, 0, 0)




vis = irr.application.IrrApp()

vis.SetWindowSize(1280, 720)

vis.SetThemeFilePrefix("chrono_irrlicht")

vis.AddWindowEvent(vis.E_WINDOWEVENT_CLOSED, 'onWindowClosed()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_RESIZED, 'onWindowResized()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_MOVED, 'onWindowMoved()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_REBUILDWINDOW, 'onWindowRebuilt()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_FOCUSED, 'onWindowFocused()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_UNFOCUSED, 'onWindowUnfocused()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_EXPOSED, 'onWindowExposed()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_MINIMIZED, 'onWindowMinimized()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_MAXIMIZED, 'onWindowMaximized()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_RESTORE, 'onWindowRestored()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_RESIZED, 'onWindowResized()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_MOVED, 'onWindowMoved()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_REBUILDWINDOW, 'onWindowRebuilt()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_FOCUSED, 'onWindowFocused()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_UNFOCUSED, 'onWindowUnfocused()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_EXPOSED, 'onWindowExposed()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_MINIMIZED, 'onWindowMinimized()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_MAXIMIZED, 'onWindowMaximized()')

vis.AddWindowEvent(vis.E_WINDOWEVENT_RESTORE, 'onWindowRestored()')




camera = vis.CreateCamera(chrono.COROLLA)

camera.SetFov(60)

camera.SetPosition(chrono.ChVectorD(0, 10, 10))

camera.SetTarget(chrono.ChVectorD(0, 0, 0))




light = vis.CreateLight()

light.SetType(irr.video.E_LIGHT_TYPE_PUNCUALR)

light.SetPosition(chrono.ChVectorD(0, 10, 10))

light.SetRange(100)




sim_time = 0

sim_duration = 10

while sim_time < sim_duration:

    

    steering_angle = np.sin(sim_time)

    driver.SetSteering(steering_angle)


    

    system.DoStepDynamics(0.01)


    

    vis.BeginScene()

    vis.DrawAll()

    vis.EndScene()


    

    sim_time += 0.01




chrono.Free(system)

vis.Run()