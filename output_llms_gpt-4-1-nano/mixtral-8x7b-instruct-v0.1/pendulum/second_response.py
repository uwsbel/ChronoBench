# Log information after 1 second of simulation time
if log_info and sys.GetChTime() > 1:
    pos_1 = pend_1.GetPos()  # Get the position of the pendulum
    print("t = ", sys.GetChTime())
    print("     ", pos_1.x, "  ", pos_1.y)
    lin_vel_1 = pend_1.GetPosDt()  # Get the linear velocity of the pendulum
    print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
    log_info = False  # Disable further logging