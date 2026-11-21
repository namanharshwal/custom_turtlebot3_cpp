# A Resource-Constrained Autonomous Mobile Robotic System for Precision 2D SLAM and Real-Time Edge Vision on Embedded 2GB SoC Architectures

**Naman Harshwal**¹*, **Pradeep Singh Chauhan**²  
¹Lead Roboticist & System Architect, Project Maintainer (Email: namanharshwal@gmail.com)  
²Project Supervisor & Systems Engineering Lead (Email: ps@robot.com)  
*Department of Autonomous Systems & Robotics Engineering*

---

### Abstract
Deploying simultaneous localization and mapping (SLAM) alongside deep neural object detection on edge micro-computing platforms has historically been hindered by the strict computational and volatile memory limits of low-cost single-board computers (SBCs). This paper presents the architecture, mathematical calibration, and empirical validation of the Custom Autonomous TurtleBot Burger 2.0, an enterprise-grade differential-drive mobile robot engineered to operate under severe hardware constraints (specifically, a Raspberry Pi 4B with only 2GB LPDDR4 RAM). We address critical real-world edge robotics challenges through four primary contributions: (1) an asymmetric hardware odometry calibration methodology that resolves a 6.8% physical gear-train tick-rate discrepancy between independent drive wheels; (2) a planar covariance regularization engine that enforces REP-103 coordinate standards and eradicates 3D covariance singularity explosions (the "yellow ellipsoid" failure mode in RViz2); (3) a zero-copy vectorized MIPI RAW10 Bayer unpacking pipeline achieving a 98.7% DMA memory reduction for YOLOv4-tiny inference throttled to 2 CPU cores; and (4) a dual-layer closed-loop anti-stall guardian combining low-level torque pulse injection with high-level odometric hysteresis backtracking. Comprehensive benchmarking shows that migrating the software stack from Python ROS 2 nodes to a native 100% Modern C++17 architecture reduces host CPU utilization by 65.4%, cuts serial telemetry latency from 14.8 ms to 0.42 ms, and stabilizes system RAM footprint at 1.18 GB without triggering out-of-memory (OOM) kernel panics.

**Keywords**: Autonomous Mobile Robots, 2D LiDAR SLAM, Embedded Edge Computing, Asymmetric Odometry Calibration, Covariance Regularization, ROS 2 Humble, Ceres Scan Matching, Deep Learning Acceleration.

---

## I. INTRODUCTION

Autonomous mobile robots (AMRs) operating in GPS-denied indoor environments rely heavily on LiDAR-based Simultaneous Localization and Mapping (SLAM) and deep neural networks for semantic environmental perception. While state-of-the-art perception frameworks perform exceptionally on workstation-class GPUs or high-end x86 platforms, deploying these capabilities onto resource-constrained edge hardware—such as the Raspberry Pi 4B with 2GB of shared memory—introduces severe bottlenecks:

1. **Volatile Memory Starvation**: Standard ROS 2 middleware stacks, SLAM Toolbox pose-graph optimization buffers, and deep learning framework memory allocations easily exceed 1.8 GB, causing aggressive operating system swapping to slow MicroSD flash storage, which induces thermal throttling and severe frame drops.
2. **Kinematic Odometry Asymmetry**: Commercial micro-metal DC gearmotors exhibit significant mechanical tolerances. Standard symmetrical kinematic formulations assume identical encoder counts per revolution (CPR), resulting in cumulative angular odometric drift (> 15 deg per linear meter) and distorted, curved mapping walls.
3. **Planar Covariance Divergence**: In commercial 9-DOF Inertial Measurement Units (such as the Bosch BNO055), unconstrained 3D pose integration yields unphysical roll and pitch covariance estimates that expand exponentially (10^6 m^2), completely occluding the visualization workspace.
4. **Mechanical Terrain Entrapment**: Low-mass differential robots traversing carpet or compliant flooring experience stall states where motor PWM commands fail to overcome static friction, causing odometric wheel slippage and graph SLAM divergence.

This paper provides a complete, mathematically grounded engineering solution to these challenges, realized on the physical Custom TurtleBot Burger 2.0 platform.

---

## II. MECHATRONIC SYSTEM ARCHITECTURE & ELECTRICAL HARNESSING

### A. Multi-Tier Chassis Topology
The robot utilizes a modular three-tier cylindrical chassis with a track width (L) of 160 mm and calibrated drive wheel radius (r) of 33 mm:
* **Tier 1 (Drive & Actuation)**: Houses dual DC micro-metal gearmotors with integrated magnetic quadrature encoders, a low-friction ball caster, and a dedicated 7.4V 2S 2500mAh Li-ion high-discharge battery pack.
* **Tier 2 (Power Management & Real-Time Microcontroller)**: Contains the star-ground power bus, dual independent LM2596 DC-DC step-down buck regulators, an Arduino Nano real-time motor controller, and bidirectional I2C/UART logic level shifters.
* **Tier 3 (Compute, Sensing & Vision)**: Houses the host Raspberry Pi 4B (Broadcom BCM2711 quad-core Cortex-A72 @ 1.5 GHz, 2GB LPDDR4), a 360° YDLidar X2 rangefinder (8 Hz scan rate, 3 kHz sample rate), a Bosch BNO055 9-DOF absolute orientation sensor, a downward-angled VL53L0X Time-of-Flight (ToF) rangefinder, and a wide-angle MIPI CSI camera.

### B. Star-Ground Topology & EMI Mitigation
Brush-commutated DC motors generate transient back-EMF spikes reaching +/- 25V, creating high-frequency inductive ground bounce across shared copper traces. To prevent I2C bus lockups and microcontroller resets, our design enforces a strict Star-Ground Architecture:
* Motor return currents travel directly to the negative terminal of the 7.4V battery pack via heavy-gauge 18 AWG stranded silicone wiring.
* Logic ground for the 5.0V microcontroller and 3.3V Raspberry Pi joins the battery negative at a single central star stud point.
* Motor terminals are shunted with 0.1 uF ceramic disc capacitors combined with Schottky flyback diodes across driver H-bridge channels.

| Subsystem Component | Supply Voltage | Peak Current | Logic Level | Interfacing Bus | Physical Pinout / Ports |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Raspberry Pi 4B (2GB) | 5.1V regulated | 2.5 A | 3.3V LVTTL | Host Processor | Pin 2/4 (5V), Pin 6 (GND) |
| Arduino Nano (MCU) | 5.0V regulated | 150 mA | 5.0V TTL | Serial Bridge | USB / UART /dev/ttyAMA0 |
| L298N / Dual H-Bridge | 7.4V V_bat | 1.8 A | 5.0V Logic | PWM / DIR | MCU D5, D6, D7, D8 |
| YDLidar X2 | 5.0V logic/motor| 450 mA | 3.3V UART | Serial (115200) | CP2102 /dev/ttyUSB0 |
| Bosch BNO055 (IMU) | 3.3V regulated | 12.5 mA | 3.3V I2C | Fast I2C (400kHz) | Pi Pin 3 (SDA), Pin 5 (SCL) |
| VL53L0X (ToF Range)| 3.3V regulated | 19 mA | 3.3V I2C | Fast I2C (Addr 0x29) | Pi Pin 3 (SDA), Pin 5 (SCL) |
| MIPI CSI Camera | 3.3V bus | 250 mA | MIPI CSI-2 | 2-Lane Camera | 15-pin FFC Ribbon Cable |

---

## III. MATHEMATICAL KINEMATICS & ASYMMETRIC CALIBRATION

### A. Differential Kinematic Model
The robot kinematics are modeled in the continuous SE(2) manifold:

[x_dot, y_dot, theta_dot]^T = [[cos(theta), 0], [sin(theta), 0], [0, 1]] * [v, omega]^T

The forward relationship between individual wheel linear velocities (v_R, v_L) and body frame velocities (v, omega) is:

v = (v_R + v_L) / 2,   omega = (v_R - v_L) / L

### B. Empirical Discovery of Asymmetric Gear-Train Resolution
In conventional robotic formulations, encoder resolution is assumed to be symmetric (CPR_L = CPR_R). However, empirical calibration under precision optical tracking revealed an inherent 6.8% asymmetry:
* Left Wheel Measured Resolution: CPR_L = 412 pulses/rev
* Right Wheel Measured Resolution: CPR_R = 440 pulses/rev

Applying symmetric CPR values caused a persistent leftward rotational drift of delta_theta = 0.28 rad/m. To eliminate this systematic error, we derive discrete distance-per-tick coefficients (delta_L, delta_R):

delta_L = (2 * pi * r) / CPR_L = (2 * pi * 0.033 m) / 412 = 5.0327e-4 m/tick

delta_R = (2 * pi * r) / CPR_R = (2 * pi * 0.033 m) / 440 = 4.7124e-4 m/tick

For discrete sampling interval delta_t = t_k - t_{k-1} with tick increments delta_N_L and delta_N_R:

delta_d = (delta_R * delta_N_R + delta_L * delta_N_L) / 2
delta_theta = (delta_R * delta_N_R - delta_L * delta_N_L) / L

Integrating over time yields the dead-reckoning pose estimate:

x_k = x_{k-1} + delta_d * cos(theta_{k-1} + delta_theta / 2)
y_k = y_{k-1} + delta_d * sin(theta_{k-1} + delta_theta / 2)
theta_k = theta_{k-1} + delta_theta

### C. Calibrated Crawl Velocity for Laser Scan Matching
LiDAR scan-matching algorithms (e.g., Ceres-based correlative scan matching in SLAM Toolbox) require sufficient geometric overlap between consecutive point clouds. The YDLidar X2 operates at rotational frequency f_s = 8 Hz (T_s = 125 ms). At typical traversal speeds (v = 0.25 m/s), the robot translates delta_x = 31.25 mm per sweep, degrading scan correlative alignment during tight turns. 

We systematically constrained maximum linear velocity to an optimal crawl speed of v_crawl = 0.07 m/s, bounding inter-scan translation to delta_x <= 8.75 mm (< 18% of the 50 mm grid cell resolution). This completely eliminates scan skewing and produces perfectly square, orthogonal wall boundaries.

---

## IV. PLANAR SENSOR FUSION & REP-103 REGULARIZATION

### A. Coordinate System Standardization & Polarity Rectification
The Bosch BNO055 reports absolute orientation using an aerospace NED (North-East-Down) convention, where yaw angle increases positively in the clockwise direction. Conversely, ROS REP-103 mandates an ENU (East-North-Up) right-handed system where counter-clockwise rotation about the Z-axis is positive. Direct ingestion of NED yaw causes odometry and laser scan transforms to rotate in opposing directions.

Our bridge node implements sign inversion and zero-datum referencing:

theta_REP103 = -theta_BNO055 + theta_offset

### B. Covariance Matrix Regularization
When 3D orientation covariance matrices are published with default uncalibrated variances or large arbitrary values (10^6), RViz2 interprets the out-of-plane pitch and roll uncertainty as massive spatial confidence ellipsoids, causing the visual display to turn completely yellow.

We enforce strict Planar 2D Covariance Regularization:

Sigma_pose = diag([0.01, 0.01, 0.00, 0.00, 0.00, 0.05])

All non-planar variances (Z-axis translation, pitch, roll) are explicitly collapsed to 0.0, instructing Ceres scan matchers and AMCL particle filters to optimize exclusively over SE(2).

---

## V. VECTORIZED MIPI CAMERA PIPELINE & EDGE INFERENCE

### A. Zero-Copy DMA Memory Optimization
Capturing 640x480 RGB video via standard Python V4L2 wrappers consumes significant memory bandwidth through intermediate buffer allocations. On a 2GB RAM platform, this triggers memory bus contention with the SLAM graph solver.

We engineered a Direct DMA Vectorized Unpacking Kernel in C++:
* The camera sensor streams in packed 10-bit raw Bayer format (V4L2_PIX_FMT_SBGGR10P).
* Five bytes of packed data encode four consecutive 10-bit pixels.
* A vectorized SIMD loop extracts the 8 most significant bits (MSBs) directly into a pre-allocated pinned memory buffer, dropping memory footprint from 74.8 MB to 0.92 MB (a 98.7% reduction).

### B. Thread Pinning & Nice Scheduling
To guarantee that deep learning inference does not starve the real-time motor controller or SLAM pose graph:
1. **CPU Core Affinity (taskset)**: Cores 0 and 1 are dedicated to hard real-time I/O (arduino_bridge, ydlidar_node, X11 TigerVNC). Cores 2 and 3 execute Nav2 global/local planners and YOLOv4-tiny.
2. **Dynamic Process Niceness**: The vision node is assigned nice 19 (lowest CPU priority), allowing the Linux CFS scheduler to preempt neural inference instantly whenever high-priority odometry interrupts occur.
3. **OpenMP Worker Throttling**: OpenCV DNN thread pooling is strictly clamped to omp_set_num_threads(2).

---

## VI. DUAL-LAYER CLOSED-LOOP ANTI-STALL RECOVERY

Low-cost differential robots frequently suffer from wheel stall when transitioning between tile and high-friction carpet. We solve this via a hierarchical Dual-Layer Anti-Stall Guardian:

```
[ Motor Current / Command Watchdog ]
                |
        Stall Detected? (Ticks == 0 for 350ms while PWM > 60)
                |
       +--------+--------+
       |                 |
[ Layer 1: Firmware ]   [ Layer 2: High-Level ROS Node ]
Micro-Torque Injection   Odometric Hysteresis Backtrack
(PWM 75 -> 105 for 80ms)  (Reverse 0.15m at v = -0.06 m/s)
       |                         |
  Stall Broken?             Rotate 45 deg Away from Obstacle
       |                         |
  Resume Normal Cruise      Re-commit to Frontier
```

---

## VII. EXPERIMENTAL BENCHMARK EVALUATIONS

The physical robot was evaluated across 25 autonomous exploration runs in a cluttered indoor environment (8.5 m x 6.2 m). We benchmarked two complete implementations: (A) standard Python ROS 2 nodes, and (B) 100% Native Modern C++17 nodes.

### Comparative Performance Metrics:
| Evaluation Metric | Python ROS 2 Stack | 100% Native C++17 Stack | Improvement |
| :--- | :--- | :--- | :--- |
| Total Host CPU Load (Average) | 88.4% | 23.0% | -65.4% |
| System RAM Footprint (Peak) | 1,894 MB (Swapping active) | 1,180 MB (Zero swap) | -37.7% |
| UART Telemetry Jitter (sigma) | 4.2 ms | 0.08 ms | -98.1% |
| SLAM Graph Update Latency | 84.2 ms | 18.6 ms | -77.9% |
| YOLOv4-tiny Inference Rate | 1.4 FPS | 4.8 FPS | +242% |
| Map Orthogonality Error | 4.8 deg deviation | 0.3 deg deviation | -93.7% |
| Recovery Sequence Success Rate| 68.0% | 96.0% | +28.0% |

---

## VIII. CONCLUSION & FUTURE DIRECTIONS

This work demonstrates that enterprise-grade autonomous navigation, metric SLAM, and real-time deep learning perception can be reliably achieved on ultra-constrained 2GB edge computing hardware. By addressing physical gear-train asymmetry, regularizing 2D sensor covariance, optimizing low-level memory pipelines, and porting critical nodes to pure C++17, the Custom TurtleBot Burger 2.0 achieves robust, deterministic performance without system crashes or thermal throttling. Future work will investigate on-device INT8 quantization for lightweight transformer-based visual SLAM.

---

## REFERENCES
1. S. Macenski, et al., "Robot Operating System 2: Design, architecture, and uses in the wild," Science Robotics, vol. 7, no. 66, 2022.
2. S. Kohlbrecher, et al., "A flexible and scalable SLAM system with full 3D motion estimation," in Proc. IEEE Int. Symp. Saf., Secur., Rescue Robot., 2011.
3. R. Kuemmerle, et al., "g2o: A general framework for graph optimization," in Proc. IEEE Int. Conf. Robot. Autom., 2011.
4. C. Wang, et al., "YOLOv4: Optimal speed and accuracy of object detection," arXiv preprint arXiv:2004.10934, 2020.
5. S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world," Journal of Open Source Software, vol. 6, no. 61, p. 2783, 2021.
6. J. Borenstein and L. Feng, "Measurement and correction of systematic odometry errors in mobile robots," IEEE Trans. Robot. Autom., vol. 12, no. 6, pp. 869-880, 1996.
7. ROS Enhancement Proposal 103, "Standard Units of Measure and Coordinate Conventions," Open Source Robotics Foundation, 2010.
8. G. Grisetti, et al., "Improved techniques for grid mapping with Rao-Blackwellized particle filters," IEEE Trans. Robot., vol. 23, no. 1, pp. 34-46, 2007.
9. D. Fox, et al., "Markov localization for mobile robots in dynamic environments," J. Artif. Intell. Res., vol. 11, pp. 391-427, 1999.
10. S. Thrun, W. Burgard, and D. Fox, Probabilistic Robotics. MIT Press, Cambridge, MA, 2005.
