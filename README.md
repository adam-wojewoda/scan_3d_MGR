# scan_3d_MGR
My masters degree project
Diagrams meant to be read with DrawIO

Project contains elements for Respberry Pi and "Server" PC
Raspberry Pi:
1. Code for for adafruit 6DOF SPI sensor (done)
2. Code for I2C sensor (done)
3. Code for linear scanner (done)
4. Code for orchestrating previous modules (to do)

PC/server:
1. Database taking sensor input (InfluxDb installed on PC)
2. IMU's sensor data analysis for creating 3d sensor movement path (Kalman filters along with basic logic)
3. Code for creating 3d points cloud from IMU path and linear scanner data containing optimizer for modifying path
4. Code for reworking cloud points into stl. (Might be part of optimizer)
5. Gui working with database and stl's
