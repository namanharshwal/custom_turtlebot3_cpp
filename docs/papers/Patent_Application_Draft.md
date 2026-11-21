# PATENT APPLICATION SPECIFICATION

## TITLE OF THE INVENTION
### SYSTEM, METHOD, AND EMBEDDED ARCHITECTURE FOR ASYMMETRIC ODOMETRY CALIBRATION, PLANAR SENSOR FUSION, AND AUTONOMOUS STALL RECOVERY IN RESOURCE-CONSTRAINED MOBILE ROBOTS

---

## INVENTORS
* **Naman Harshwal** (Lead Inventor & Roboticist, Resident of India, Email: namanharshwal@gmail.com)
* **Pradeep Singh Chauhan** (Co-Inventor & Systems Lead, Resident of India, Email: ps@robot.com)

---

## ABSTRACT OF THE DISCLOSURE
An autonomous mobile robotic system, embedded architecture, and computer-implemented control method for high-fidelity planar Simultaneous Localization and Mapping (SLAM) and real-time deep neural vision under severe volatile memory and computational constraints. The system comprises a differential-drive mechatronic chassis driven by independent DC micro-gear actuators, a host single-board computer having limited random-access memory (2GB), a planar rangefinder, an absolute orientation sensor, and an optical imaging sensor. The system mitigates severe systematic odometric drift by executing an asymmetric distance-per-tick kinematic calculation that compensates for a measured 6.8% gear-train resolution discrepancy between independent wheels (CPR_L = 412, CPR_R = 440). A sensor fusion engine regularizes spatial covariance matrices to eliminate out-of-plane 3D uncertainty divergence while enforcing ROS REP-103 right-handed coordinate polarity. A low-overhead vision pipeline employs a vectorized 10-bit packed Bayer direct memory access (DMA) kernel that reduces active image memory consumption by 98.7% for CPU-pinned neural object detection. A closed-loop dual-stage anti-stall controller injects localized micro-torque current pulses and executes odometric hysteresis backtracks to eliminate terrain entrapment without destabilizing graph SLAM pose graphs.

---

## FIELD OF THE INVENTION
The present invention relates generally to autonomous mobile robotics, embedded systems, and real-time computer vision. More specifically, the invention relates to methods, architectures, and algorithms for eliminating systematic odometry drift, regularizing inertial covariance matrices, optimizing memory bandwidth, and executing autonomous stall recovery on resource-constrained embedded computing platforms.

---

## BACKGROUND OF THE INVENTION AND PRIOR ART
Autonomous mobile robots (AMRs) operating in GPS-denied indoor environments rely on planar LiDAR rangefinders, dead-reckoning wheel odometry, and inertial measurement units (IMUs) to estimate pose and construct occupancy grid maps using graph-based SLAM algorithms. In commercial applications, deploying such systems on low-cost single-board computers (such as the Raspberry Pi 4B with 2GB LPDDR4 memory) introduces severe technical challenges:

1. **Systematic Gear-Train Asymmetry**: Symmetrical kinematic models assume identical counts per revolution (CPR) for left and right wheels. Due to micro-machining tolerances in low-cost planetary and spur gearboxes, actual tick rates vary significantly (empirically measured at 6.8% between channels). In prior art systems, this causes continuous angular dead-reckoning drift (> 15 deg per linear meter), yielding curved, distorted wall boundaries in graph SLAM.
2. **Out-of-Plane Covariance Singularity**: Low-cost 9-DOF IMUs output orientation estimates governed by internal proprietary Kalman filters configured for 3D aerial or automotive applications. In planar 2D mobile robots, unconstrained roll and pitch uncertainties diverge, publishing artificially massive covariance values (e.g., 10^6 m^2) that trigger visual render failures (the "yellow screen" anomaly in RViz2) and corrupt graph optimization cost functions.
3. **Volatile Memory Starvation & Flash Thrashing**: Standard image acquisition stacks perform multiple RGB color conversions in heap memory. In edge systems with 2GB RAM, deep learning inference (such as YOLO object detection) triggers out-of-memory kernel panics and aggressive swap paging on flash memory, degrading real-time control loops.
4. **Mechanical Terrain Stalls**: When low-mass robots encounter transitions between polished flooring and high-friction carpets, static friction frequently exceeds cruise motor torque. In prior art systems, this causes wheel lockup, unmeasured odometric slippage, and catastrophic divergence of the SLAM map.

There is therefore a critical need for an integrated system and method that overcomes these systemic limitations on low-cost edge robotics hardware.

---

## SUMMARY OF THE INVENTION
The present invention provides a comprehensive system, embedded architecture, and control method that solves each of the aforementioned limitations.

In a first aspect, the invention provides an asymmetric odometric calibration system that independently scales encoder increments using discrete Left and Right distance-per-tick coefficients, bounding cumulative rotational drift below 0.02 rad/m.

In a second aspect, the invention provides a planar sensor covariance regularization engine that constrains 6x6 spatial pose covariance to SE(2), eliminating out-of-plane divergence and converting aerospace NED orientation to REP-103 ENU coordinate frames.

In a third aspect, the invention provides a vectorized direct memory access (DMA) Bayer unpacking kernel that bypasses user-space RGB memory duplication, reducing image buffer memory by 98.7% and enabling CPU-pinned deep neural inference at niceness priority 19.

In a fourth aspect, the invention provides a hierarchical dual-stage anti-stall guardian combining low-level hardware torque pulse injection with high-level odometric hysteresis backtracking.

In a fifth aspect, the invention provides a star-ground electrical isolation topology that eliminates motor back-EMF ground bounce, preventing I2C bus lockup and serial telemetry packet corruption.

---

## BRIEF DESCRIPTION OF THE DRAWINGS
* **FIG. 1** is a system block diagram illustrating the multi-tier mechatronic architecture, star-ground electrical bus, and sensor interfaces.
* **FIG. 2** is a schematic diagram of the differential-drive kinematic geometry showing asymmetric wheel resolution vectors.
* **FIG. 3** is a flowchart depicting the asymmetric distance-per-tick odometry calculation and dead-reckoning pose propagation.
* **FIG. 4** is a matrix diagram illustrating the SE(2) planar covariance regularization and REP-103 coordinate sign inversion.
* **FIG. 5** is a memory architecture diagram illustrating the zero-copy vectorized 10-bit packed Bayer DMA unpacking pipeline.
* **FIG. 6** is a state machine diagram showing the dual-stage closed-loop anti-stall recovery sequence.
* **FIG. 7** is a timing diagram comparing CPU utilization and telemetry jitter between the 100% C++17 native implementation and conventional Python nodes.
* **FIG. 8** is an occupancy grid comparison demonstrating distortion-free metric mapping achieved by the present invention.

---

## DETAILED DESCRIPTION OF PREFERRED EMBODIMENTS

### 1. Asymmetric Odometry Calibration Architecture
Referring to FIG. 2 and FIG. 3, the robot comprises a left drive wheel and a right drive wheel separated by track width L = 0.160 m, each having radius r = 0.033 m. Encoders measure shaft rotation with asymmetric measured counts per revolution:
* Left Wheel: CPR_L = 412 pulses/rev
* Right Wheel: CPR_R = 440 pulses/rev

The embedded processor computes discrete scale coefficients:
delta_L = (2 * pi * r) / CPR_L = 5.03268e-4 m/tick
delta_R = (2 * pi * r) / CPR_R = 4.71239e-4 m/tick

During each sampling epoch delta_t, the displacement delta_d and angular rotation delta_theta are computed as:
delta_d = (delta_R * Delta N_R + delta_L * Delta N_L) / 2
delta_theta = (delta_R * Delta N_R - delta_L * Delta N_L) / L

### 2. Planar Covariance Regularization Engine
Referring to FIG. 4, absolute heading theta_IMU from the BNO055 sensor is transformed to satisfy ROS REP-103 ENU standards:
theta_REP103 = -theta_IMU + theta_calib

The 6x6 spatial covariance matrix Sigma_pose is regularized to eliminate out-of-plane divergence:
Sigma_pose = diag([0.01, 0.01, 0.00, 0.00, 0.00, 0.05])
Where variances for z-translation, pitch, and roll are fixed at 0.0, constraining optimization strictly to planar SE(2).

### 3. Vectorized Zero-Copy Vision Pipeline
Referring to FIG. 5, raw optical data is captured via MIPI CSI-2 in 10-bit packed Bayer format (`V4L2_PIX_FMT_SBGGR10P`). A vectorized C++ kernel executes:
* Direct DMA ingestion into a pre-allocated single-buffer ring.
* SIMD extraction of 8-bit MSBs across 4 pixels per 5-byte block.
* Direct memory passing to an OpenCV DNN tensor throttled to 2 OpenMP threads and scheduled at `nice 19` on isolated CPU cores 2 and 3.

### 4. Dual-Stage Anti-Stall Guardian
Referring to FIG. 6, when encoder velocity delta_d / delta_t drops to zero for 350 ms while commanded PWM > 60:
* **Stage 1**: Firmware injects an 80 ms micro-torque pulse (PWM 75 -> 105).
* **Stage 2**: If stall condition persists, the system commands an autonomous reverse displacement of 0.15 m at v = -0.06 m/s, followed by a 45 deg rotational yaw escape.

---

## PATENT CLAIMS

### WE CLAIM:

1. **An autonomous mobile robotic system comprising:**
   * a differential-drive chassis comprising first and second independently actuated drive wheels separated by a baseline width;
   * first and second rotary encoders operatively coupled to said first and second drive wheels, wherein said first encoder exhibits a first count-per-revolution (CPR) resolution and said second encoder exhibits a second CPR resolution distinct from said first CPR resolution;
   * an embedded single-board computing platform having random-access memory of 2 gigabytes or less;
   * a planar laser rangefinder and an absolute orientation sensor operatively coupled to said computing platform; and
   * a memory storing instructions that, when executed by said computing platform, cause the computing platform to:
     * compute independent first and second distance-per-tick scale factors based on said first and second distinct CPR resolutions;
     * calculate dead-reckoning linear displacement and angular rotation from encoder tick increments scaled by said independent factors;
     * regularize spatial covariance estimates of said absolute orientation sensor by zeroing out-of-plane variances; and
     * execute graph-based simultaneous localization and mapping (SLAM) bounded by a calibrated maximum crawl velocity.

2. The system of claim 1, wherein said first CPR resolution is approximately 412 counts per revolution and said second CPR resolution is approximately 440 counts per revolution, defining a resolution asymmetry of approximately 6.8%.

3. The system of claim 1, wherein said calibrated maximum crawl velocity is bounded within the range of 0.05 m/s to 0.08 m/s, such that inter-scan linear translation between successive laser sweeps is strictly less than 20% of an occupancy grid cell resolution.

4. The system of claim 1, wherein regularizing spatial covariance estimates comprises assigning non-zero variances exclusively to planar x-translation, planar y-translation, and planar yaw-rotation, and assigning zero variance to vertical z-translation, pitch-rotation, and roll-rotation.

5. The system of claim 1, wherein said absolute orientation sensor provides orientation data in an aerospace North-East-Down (NED) coordinate convention, and wherein said instructions further invert the mathematical sign of said orientation data to conform to an East-North-Up (ENU) right-handed coordinate convention.

6. The system of claim 1, further comprising a downward-oriented Time-of-Flight (ToF) rangefinder configured to detect drop-offs and low-profile floor obstacles below a scanning plane of said planar laser rangefinder.

7. The system of claim 1, further comprising an electrical power distribution system comprising a star-ground grounding block, wherein motor return currents are isolated from digital logic ground returns to eliminate inductive ground bounce.

8. The system of claim 1, further comprising a camera sensor streaming packed 10-bit raw Bayer image frames to said computing platform via a direct memory access (DMA) ring buffer.

9. The system of claim 8, wherein said computing platform executes a vectorized unpacking kernel that converts said packed 10-bit raw Bayer frames into an 8-bit image tensor without allocating intermediate heap memory buffers, reducing active image memory footprint by at least 95%.

10. The system of claim 9, wherein deep neural network inference on said image tensor is pinned to a dedicated subset of CPU cores and assigned a low-priority scheduling niceness of 19.

11. **A computer-implemented method for autonomous robotic navigation on resource-constrained embedded hardware, comprising:**
    * receiving first encoder ticks from a first drive wheel having a first CPR and second encoder ticks from a second drive wheel having a second CPR differing from said first CPR by at least 5%;
    * independently scaling said first and second encoder ticks by respective first and second distance-per-tick coefficients to compute asymmetric wheel displacements;
    * calculating dead-reckoning pose estimates on an SE(2) planar manifold;
    * fusing planar orientation data from an inertial sensor while enforcing zero covariance on non-planar degrees of freedom;
    * constructing an occupancy grid map via non-linear least squares scan matching at an operational crawl speed bounded below 0.08 m/s; and
    * executing a dual-layer anti-stall sequence upon detecting zero encoder movement during active motor drive commands.

12. The method of claim 11, wherein said dual-layer anti-stall sequence comprises:
    * executing a low-level firmware torque pulse increasing motor pulse-width-modulation (PWM) by at least 25% for a duration of 50 to 120 milliseconds; and
    * upon failure of said torque pulse, commanding a high-level reverse translation of at least 0.10 meters followed by an angular yaw rotation.

13. The method of claim 11, wherein all software processes for motor bridging, laser scan processing, and graph SLAM are compiled natively in Modern C++17 to eliminate garbage collection pauses and dynamic memory reallocations.

14. The method of claim 11, wherein serial communication between said computing platform and a motor controller is executed via direct POSIX termios system calls with hardware UART latency bounded below 1.0 millisecond.

15. The method of claim 11, wherein an inscribed robot radius parameter of 0.20 meters is enforced across global and local navigation costmaps to maintain collision-free obstacle clearance around a chassis having a 0.16 meter track baseline.

16. **A non-transitory computer-readable medium storing instructions for controlling an autonomous mobile robot, the instructions comprising:**
    * code for reading raw encoder pulse counts from independent left and right wheel actuators;
    * code for multiplying left pulse counts by a calibrated scale factor of 5.0327e-4 meters per count and right pulse counts by a calibrated scale factor of 4.7124e-4 meters per count;
    * code for propagating robot odometry using the scaled pulse counts;
    * code for filtering inertial measurement unit data to invert clockwise yaw into counter-clockwise yaw;
    * code for conditioning a 6x6 spatial covariance matrix such that pitch, roll, and z-axis variances are zero; and
    * code for executing scan-matching SLAM at a constrained linear speed not exceeding 0.07 m/s.

17. The computer-readable medium of claim 16, further comprising code for capturing 10-bit packed Bayer camera frames directly via DMA and unpacking the 8 most significant bits into a neural inference buffer without user-space buffer duplication.

18. The computer-readable medium of claim 17, wherein said neural inference is executed by a lightweight YOLOv4-tiny convolutional neural network restricted to two OpenMP execution threads.

19. The computer-readable medium of claim 16, further comprising code for detecting motor stall when encoder tick displacement remains zero for greater than 350 milliseconds during an active driving state.

20. The computer-readable medium of claim 19, further comprising code for triggering an autonomous recovery sequence comprising firmware micro-torque injection and odometric backtrack navigation.
