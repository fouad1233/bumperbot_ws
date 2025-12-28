# Odometry Overview

This note summarizes how we estimate a robot's pose from wheel encoders, common sources of error, and how to mitigate drift.

![Robot pose frames](image/Odometry/1766916980779.png)

We describe the robot pose relative to:

- **O**: odom frame (start point)
- **R**: current robot frame
- **M**: map frame (global reference)

This document focuses on odometry within the odom frame (`O→R`). The relation to the map frame (`R→M`) is covered later in mapping/localization.

![Frames relationship](image/Odometry/1766917016551.png)

## Local Odometry (Dead Reckoning)

By integrating velocity over time we can track the robot pose relative to the start. Wheel encoders count steps and, combined with the wheel radius, let us convert rotations into linear and angular motion.

![Dead reckoning](image/Odometry/1766917251045.png)

## Wheel Encoders

- **Incremental encoders** emit square waves as a slotted disk interrupts a light source. Frequency gives speed, but direction requires quadrature channels. They do not provide an absolute position after power loss.
- **Absolute encoders** assign a unique binary code to each angular section, so a single reading reveals both position and direction.

![Incremental encoder principle](image/Odometry/1766917661292.png)
![Incremental encoder waveform](image/Odometry/1766917990494.png)
![Absolute encoder disk](image/Odometry/1766918944978.png)
![Binary coding for absolute encoder](image/Odometry/1766919044306.png)

More channels → higher resolution → better pose estimation. Accurate wheel radius calibration is equally important.

## Odometry Error Sources

- Wheel radius mismatch or mounting tolerances
- Poor ground contact or slippage
- Quantization from low-resolution encoders
- Integration drift that accumulates over distance/time

![Slippage example](image/Odometry/1766919216658.png)
![Uneven ground](image/Odometry/1766920486150.png)

## Improving Accuracy

Two complementary strategies reduce drift and uncertainty:

1) **Sensor fusion**: combine wheel odometry with other sensors (e.g., IMU, lidar, visual odometry) to smooth noise and detect slips.
2) **Global localization**: align odometry to a map of the environment to periodically correct accumulated error (covered in the localization and mapping course).

## Differential Inverse Kinematics

![1766920849322](image/Odometry/1766920849322.png)

V = (s/t) where s is the distance and t is time

![1766920967918](image/Odometry/1766920967918.png)

![1766921049772](image/Odometry/1766921049772.png)


![1766921141736](image/Odometry/1766921141736.png)
