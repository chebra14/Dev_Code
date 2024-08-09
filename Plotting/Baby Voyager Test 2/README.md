Baby voyager test 2 is to try and detect yaw related slipping. If the yaw does not match the model, then it is slipping.

Planar sliding is not implelent (A velocity estimate is probably needed).

Format:
time(ms),accel_x(m/s^2),accel_y(m/s^2),accel_z(m/s^2),gyro_x(rad/s),gyro_y(rad/s),gyro_z(rad/s),W_speed_right(pulse count),W_speed_left(pulse count)
yaw_model(rad/s)

Slip Test 3 - threshold = 0.6 rad/s
Slip Test 4 - threshold = 0.5 rad/s
