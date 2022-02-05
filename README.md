# scan_3d_MGR
My masters degree project
Diagrams meant to be read with DrawIO

Project contains elements for Respberry Pi and "Server" PC
Raspberry Pi:
1. Code for for adafruit 6DOF SPI sensor (done)
2. Code for I2C sensor (done)
3. Code for linear scanner (done)
4. Code for orchestrating previous modules (done)

PC/server:
1. Database taking sensor input (InfluxDb installed on PC)
2. IMU's sensor data analysis for creating 3d sensor movement path (Kalman filters along with basic logic)
   1. Translation of coordinates
   2. Straight integration
   3. Kalman filtering
   4. Structure for exporting scanner position on path 
3. Code for creating 3d points cloud from IMU path and linear scanner data containing optimizer for modifying path
   1. Getting scanner position to measurement structure
   2. Translation of scanner measurements with path position (gives points cloud)
   3. Cloud optimization
      1. Measurement for local quality (Voxels might be needed)
      2. Desired direction of points movement (In order to increase the quality)
      3. Desired direction of scanner movement (As aggregation of points)
      4. The optimizer itself - nudging the position derivatives to improve the scanner position 
         in direction of it's desired movement
4. Code for reworking cloud points into stl. (Might be part of optimizer)
5. Gui working with database and STLs
