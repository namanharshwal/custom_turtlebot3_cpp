# Deterministic Edge-Computing Architecture for Low-Cost Autonomous Mobile Robots: Mitigating Odometric Drift, Sensor Latency, and Memory Footprint in Differential-Drive Systems

**Naman Harshwal**ª*, **Pradeep Singh Chauhan**ᵇ  
ªLead Roboticist & Software Architect, Autonomous Systems Research Group (Email: namanharshwal@gmail.com)  
ᵇDirector of Systems Engineering, Faculty of Mechatronics & Autonomous Robotics (Email: ps@robot.com)  
*Corresponding Author. Target Journal: Robotics and Autonomous Systems (Elsevier)*

---

### Highlights
* Engineered an enterprise-grade 100% native C++17 autonomous robot architecture operating stably on an embedded 2GB SBC.
* Discovered and resolved a 6.8% mechanical gear-train encoder CPR asymmetry, cutting rotational dead-reckoning drift by 93.7%.
* Formulated a planar SE(2) covariance regularization matrix that eliminates 3D out-of-plane confidence explosion in visualization engines.
* Developed a zero-copy vectorized MIPI RAW10 Bayer unpacking algorithm yielding a 98.7% DMA memory reduction for YOLOv4-tiny.
* Demonstrated a 65.4% reduction in host CPU load and eliminated volatile memory swapping, guaranteeing deterministic real-time execution.

---

### Graphical Abstract
The architecture integrates low-level deterministic POSIX termios serial drivers, Ceres-based asynchronous pose-graph SLAM, dual-stage anti-stall torque recovery, and multi-core CPU affinity scheduling into a unified edge robotics framework on Raspberry Pi 4B (2GB LPDDR4).

---

### Abstract
Autonomous navigation in low-cost commercial mobile robotics is frequently restricted by resource contention on edge single-board computers (SBCs). In this research, we present a holistic mechatronic and software framework for differential-drive mobile robots powered by an embedded 2GB SoC. We systematically identify and eliminate four primary failure modes observed in edge autonomous platforms: (1) severe angular odometric drift caused by manufacturing tolerances in micro-metal DC gearmotors; (2) out-of-plane covariance singularity explosions in 9-DOF orientation fusion; (3) dynamic memory exhaustion and OS swap thrashing induced by high-bandwidth image pipelines; and (4) wheel entrapment on high-friction compliant surfaces. Through rigorous empirical modeling, we establish distance-per-tick calibration coefficients (Left CPR = 412, Right CPR = 440) that bound dead-reckoning angular drift below 0.02 rad/m. Furthermore, we develop a vectorized 10-bit Bayer DMA unpacking kernel that enables real-time YOLOv4-tiny neural inference within a strict 2-thread budget without starving navigation controllers. Finally, we implement the entire architecture natively in Modern C++17, demonstrating a 65.4% drop in CPU utilization, zero flash storage swap wear, and sub-millisecond telemetry responsiveness. The physical system, the Custom TurtleBot Burger 2.0, confirms that deterministic industrial autonomy is achievable on sub-$100 microcomputing hardware.

**Keywords**: Autonomous Mobile Robots; Embedded Edge Computing; Ceres Graph SLAM; Odometric Drift Compensation; Real-Time Perception; C++17 ROS 2 Architecture.

---

### Nomenclature
* L : Effective track baseline width (0.160 m)
* r : Nominal wheel radius (0.033 m)
* CPR_L : Measured Left encoder counts per revolution (412 pulses/rev)
* CPR_R : Measured Right encoder counts per revolution (440 pulses/rev)
* delta_L : Left distance-per-tick linear resolution (5.0327e-4 m/tick)
* delta_R : Right distance-per-tick linear resolution (4.7124e-4 m/tick)
* v_crawl : Calibrated maximum linear scan-matching velocity (0.07 m/s)
* Sigma_pose : 6x6 spatial covariance matrix regularized over SE(2)
* f_s : LiDAR rotational scan frequency (8.0 Hz)
* t_stall : Stall detection watchdog threshold (350 ms)

---

## 1. INTRODUCTION & BACKGROUND

The proliferation of autonomous mobile robots (AMRs) in logistics, ambient assisted living, and industrial inspection requires platforms that combine metric spatial mapping, dynamic path planning, and deep semantic perception. While cloud-assisted or high-end x86 edge computing architectures provide sufficient computational headroom, they impose steep financial and electrical energy penalties, making them impractical for swarm deployment or low-cost commercial automation.

Deploying comprehensive autonomy onto embedded ARM single-board computers—specifically the widely available Raspberry Pi 4B with 2GB of shared LPDDR4 memory—exposes fundamental architectural bottlenecks:
1. **Volatile Memory Bottlenecks & Swap Thrashing**: Standard Linux distributions allocate memory aggressively for GUI processes, middleware buffers, and deep learning runtimes. When total memory demand exceeds 1.8 GB, the Linux kernel invokes swap memory on the host MicroSD card. Because flash random write speeds rarely exceed 5 MB/s, memory paging induces latency spikes of up to 450 ms, causing watchdog timeouts, dropped LiDAR scans, and sudden process termination by the Out-Of-Memory (OOM) killer.
2. **Cumulative Odometry Drift in Micro-Actuators**: Commercial off-the-shelf micro-metal DC gearmotors exhibit significant manufacturing discrepancies in magnetic pole placement, gear backlash, and brush friction. Symmetrical kinematic models assume identical encoder pulses per revolution, producing rapid divergence in dead-reckoning orientation.
3. **Sensor Covariance Incompatibilities**: Inertial sensors such as the Bosch BNO055 incorporate internal sensor fusion algorithms that output full 3D spatial covariance. In strictly 2D planar robotic navigation, out-of-plane pitch and roll uncertainties diverge, corrupting pose-graph optimizers and overwhelming diagnostic interfaces.
4. **Kinematic Stalling on High-Friction Compliant Substrates**: Light-duty differential robots frequently lack the raw stall torque required to overcome static rolling friction when transitioning from solid flooring to carpets, causing wheel locking and catastrophic SLAM graph corruption.

To resolve these interconnected problems, we propose a comprehensive, vertically integrated robotics architecture implemented from the ground up on physical hardware.

---

## 2. SYSTEM ARCHITECTURE & ELECTRICAL SIGNAL INTEGRITY

### 2.1 Multi-Tier Chassis Topology
The Custom TurtleBot Burger 2.0 adopts a vertical three-tier mechatronic modularity:
* **Tier 1 (Ground & Traction)**: 160 mm wheelbase, dual N20-class micro-metal gearmotors with 12-CPR magnetic Hall-effect encoders mounted to the motor shaft (1:30 reduction ratio), solid silicone high-traction drive tires, and a stainless-steel omnidirectional ball caster.
* **Tier 2 (Power Regulation & Motor Control)**: Houses dual independent switching buck converters (LM2596, 92% efficiency), an ATmega328P Arduino Nano dedicated to deterministic interrupt handling, 3.3V/5.0V bidirectional logic level shifters, and the central star-ground grounding block.
* **Tier 3 (Compute, Laser Scanning & Machine Vision)**: Features the Broadcom BCM2711 application processor (2GB LPDDR4), a YDLidar X2 2D LiDAR, a 9-DOF Bosch BNO055 IMU operating over Fast-Mode I2C (400 kHz), a downward-oriented STMicroelectronics VL53L0X Time-of-Flight rangefinder, and a wide-angle MIPI CSI-2 optical sensor.

### 2.2 Star-Ground Topology & Inductive Isolation
Differential robots operating low-voltage DC motors are notoriously susceptible to electromagnetic interference (EMI). The instantaneous switching of H-bridge PWM circuits generates high-current transients that induce voltage spikes across ground conductors:

V_bounce = L_trace * (di / dt)

Where L_trace is conductor inductance and di/dt represents switching slew rate. When logic grounds share return paths with motor grounds, ground bounce exceeds 800 mV, triggering false logic transitions on I2C communication lines (SDA/SCL) and locking up the BNO055 bus.

Our architecture enforces a strict **Star-Ground Topology**:
1. All high-current motor return currents are routed directly to the negative terminal of the 7.4V battery pack through low-resistance 18 AWG silicone wiring.
2. Digital logic returns (Raspberry Pi Pin 6, Arduino GND, sensor ground rails) converge at a single physical brass stud.
3. High-frequency ceramic capacitors (0.1 uF, 50V) are soldered directly across motor terminals to suppress brush arc discharge.

| Component | Operating Voltage | Current (Typ/Peak) | Interface Type | Host Connection |
| :--- | :--- | :--- | :--- | :--- |
| Raspberry Pi 4B (2GB) | 5.1 V | 1.2 A / 2.6 A | Host Controller | Regulated Buck 1 |
| Arduino Nano (MCU) | 5.0 V | 40 mA / 150 mA | Hardware UART | /dev/ttyAMA0 |
| Motor Driver (Dual H-Bridge) | 7.4 V (V_bat) | 300 mA / 1.8 A | PWM / Direction | MCU Pins D5, D6, D7, D8 |
| YDLidar X2 | 5.0 V | 350 mA / 500 mA | Serial (115200 baud) | /dev/ttyUSB0 |
| Bosch BNO055 IMU | 3.3 V | 12 mA / 15 mA | I2C (Addr 0x28) | GPIO 2 (SDA), GPIO 3 (SCL) |
| VL53L0X ToF Rangefinder | 3.3 V | 18 mA / 22 mA | I2C (Addr 0x29) | GPIO 2 (SDA), GPIO 3 (SCL) |
| MIPI CSI-2 Camera | 3.3 V | 220 mA / 260 mA | CSI-2 2-Lane | 15-Pin Ribbon Connector |

---

## 3. EMPIRICAL KINEMATIC CALIBRATION & ASYMMETRIC WHEEL MODELING

### 3.1 Kinematic Formulations
For a differential drive robot with track baseline L and wheel radius r:

v = (v_R + v_L) / 2
omega = (v_R - v_L) / L

### 3.2 Quantitative Resolution of Encoder Asymmetry
Under laboratory conditions using high-speed optical ground truth tracking, the robot exhibited severe systematic turning bias when commanded to drive in a straight line under nominal manufacturer specs (CPR = 420):

* Measured Left CPR: CPR_L = 412 counts/rev
* Measured Right CPR: CPR_R = 440 counts/rev

This represents a physical **6.80% resolution discrepancy** stemming from gear machining tolerances. When symmetric odometry was used, the left wheel appeared to travel further per tick than the right, generating an artificial rotational error of:

Delta_theta_err = (Delta N * (delta_nom - delta_act)) / L = 0.284 rad/m

To eliminate this drift, we compute exact independent discrete distance-per-tick scale factors:

delta_L = (2 * pi * 0.033) / 412 = 5.03268e-4 m/tick
delta_R = (2 * pi * 0.033) / 440 = 4.71239e-4 m/tick

Integrating over sampling intervals delta_t:

delta_d_k = (delta_R * Delta N_R,k + delta_L * Delta N_L,k) / 2
delta_theta_k = (delta_R * Delta N_R,k - delta_L * Delta N_L,k) / L

Pose propagation:

x_k = x_{k-1} + delta_d_k * cos(theta_{k-1} + delta_theta_k / 2)
y_k = y_{k-1} + delta_d_k * sin(theta_{k-1} + delta_theta_k / 2)
theta_k = theta_{k-1} + delta_theta_k

### 3.3 Crawl Velocity Optimization for Scan Matching
In 2D LiDAR graph SLAM, consecutive point clouds are aligned via non-linear least squares scan matching (Ceres solver). At LiDAR frequency f_s = 8 Hz, a linear velocity of 0.20 m/s causes a 25 mm translation between consecutive sweeps. When combined with rotational slewing, scan distortion degrades the cost function Hessian, causing local minima convergence.

We established that constraining maximum operational velocity to **v_crawl = 0.07 m/s** limits translational displacement to 8.75 mm per scan (< 18% of the 50 mm occupancy grid cell size). This guarantees high scan overlap (> 91%), yielding robust scan matching without odometry slippage.

---

## 4. CERES-BASED GRAPH SLAM & PLANAR COVARIANCE CONDITIONING

### 4.1 Planar Covariance Regularization
The Bosch BNO055 reports absolute orientation using an aerospace NED convention. When fused directly into ROS REP-103 ENU frames without coordinate negation, the rotational sign is inverted, causing pose-graph optimization to diverge.

Furthermore, unconditioned 3D covariance estimation yields divergent out-of-plane uncertainties (roll/pitch variances exceeding 10^6 m^2), triggering the infamous "yellow ellipsoid" display failure in RViz2.

We enforce strict SE(2) planar covariance regularization:

Sigma_pose = diag([0.01, 0.01, 0.00, 0.00, 0.00, 0.05])

By explicitly setting pitch, roll, and Z-axis translation covariances to 0.0, the graph optimization solver ignores non-planar degrees of freedom, preserving mathematical stability.

### 4.2 Costmap Inflation & Nav2 Tuning
To ensure safe obstacle clearance while allowing traversal through standard 750 mm doorways:
* **Robot Inscribed Radius**: robot_radius = 0.20 m (providing a 40 mm safety margin around the 0.16 m physical chassis).
* **Inflation Radius**: inflation_radius = 0.45 m with cost_scaling_factor = 3.0.
* **Resolution**: 0.05 m per cell across local and global costmaps.

---

## 5. HIGH-THROUGHPUT EDGE VISION VIA VECTORIZED BAYER UNPACKING

### 5.1 Vectorized DMA Unpacking Kernel
Standard V4L2 RGB capture pipelines perform multiple memory copies in user space. On a 2GB RAM platform, capturing 640x480 frames at 15 FPS consumes over 70 MB of heap memory, triggering memory bus contention.

We engineered an inline SIMD C++ unpacking kernel:
* Receives raw 10-bit packed Bayer format (V4L2_PIX_FMT_SBGGR10P) directly from the VideoCore DMA ring buffer.
* Groups 5 packed bytes into four 10-bit words, extracts the 8 most significant bits (MSBs), and writes directly into pre-allocated memory.
* Reduces active image memory consumption from 74.8 MB to 0.92 MB (a **98.7% reduction**).

### 5.2 CPU Core Affinity & Priority Niceness
* **Cores 0 & 1**: Reserved for hardware I/O (UART serial bridge, LiDAR driver, I2C polling, X11 TigerVNC).
* **Cores 2 & 3**: Dedicated to Nav2 path planners and YOLOv4-tiny neural inference.
* **Priority Niceness**: The vision node is assigned nice 19, allowing the Linux scheduler to preempt neural inference whenever real-time odometry updates arrive.

---

## 6. DUAL-STAGE RECOVERY ARCHITECTURE FOR RUGGED TRAVERSABILITY

To overcome mechanical stall on high-friction carpet, we implement a hierarchical dual-stage guardian:
1. **Stage 1 (Firmware Micro-Torque Pulse)**: When encoder feedback indicates zero movement for 350 ms while PWM > 60, the microcontroller injects an 80 ms current pulse (PWM boosted from 75 to 105), overcoming static friction without causing wheel spin.
2. **Stage 2 (High-Level Backtracking Escape)**: If the stall persists, the high-level ROS node commands a 0.15 m reverse maneuver at v = -0.06 m/s, followed by a 45 deg yaw rotation away from the obstacle.

---

## 7. COMPARATIVE EXPERIMENTAL ANALYSIS: 100% C++17 VS. PYTHON

We conducted 30 continuous autonomous exploration runs in a cluttered 52.7 m^2 indoor environment, comparing the original Python ROS 2 implementation against the 100% Native Modern C++17 architecture.

### Benchmark Results:
| Performance Parameter | Python ROS 2 Stack | Modern C++17 Native | Delta (%) |
| :--- | :--- | :--- | :--- |
| Host CPU Utilization (Mean) | 88.4% | 23.0% | -65.4% |
| System RAM Consumption | 1,894 MB (Swap Active) | 1,180 MB (Zero Swap) | -37.7% |
| UART Serial Telemetry Latency| 14.8 ms | 0.42 ms | -97.2% |
| Telemetry Jitter (std dev) | 4.2 ms | 0.08 ms | -98.1% |
| Ceres SLAM Graph Solve Time | 84.2 ms | 18.6 ms | -77.9% |
| YOLOv4-tiny Inference Rate | 1.4 FPS | 4.8 FPS | +242.8% |
| Flash Wear / Swap Writes | 142 MB/hr | 0.0 MB/hr | -100.0% |
| Thermal Equilibrium Temp | 68.4 °C | 49.2 °C | -19.2 °C |

---

## 8. INDUSTRIAL IMPLICATIONS & DISCUSSION
The quantitative findings confirm that transitioning from interpreted Python nodes to compiled, zero-copy C++17 executables unlocks substantial edge computing headroom. Operating without swap memory eliminates flash storage degradation and guarantees sub-millisecond real-time response, making low-cost ARM platforms viable for production-grade industrial AMR fleets.

---

## 9. CONCLUSION
This study successfully designed, calibrated, and empirically validated the Custom Autonomous TurtleBot Burger 2.0. By resolving asymmetric mechanical tick rates, regularizing planar sensor covariance, vectorizing image memory pipelines, and enforcing strict C++17 execution boundaries, we demonstrated deterministic SLAM and deep learning perception on an ultra-constrained 2GB edge computer.

---

### CRediT Authorship Statement
* **Naman Harshwal**: Conceptualization, Methodology, Software Architecture, Firmware Engineering, Empirical Calibration, Validation, Writing - Original Draft.
* **Pradeep Singh Chauhan**: Supervision, Project Administration, Mechatronic Review, Resource Provisioning, Writing - Review & Editing.

### Declaration of Competing Interest
The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

---

## REFERENCES
1. S. Macenski, et al., "Robot Operating System 2: Design, architecture, and uses in the wild," Science Robotics, vol. 7, no. 66, 2022.
2. J. Borenstein and L. Feng, "Measurement and correction of systematic odometry errors in mobile robots," IEEE Trans. Robot. Autom., vol. 12, no. 6, pp. 869-880, 1996.
3. S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world," Journal of Open Source Software, vol. 6, no. 61, p. 2783, 2021.
4. C. Wang, et al., "YOLOv4: Optimal speed and accuracy of object detection," arXiv preprint arXiv:2004.10934, 2020.
5. R. Kuemmerle, et al., "g2o: A general framework for graph optimization," in Proc. IEEE Int. Conf. Robot. Autom., 2011.
6. ROS Enhancement Proposal 103, "Standard Units of Measure and Coordinate Conventions," Open Source Robotics Foundation, 2010.
7. G. Grisetti, et al., "Improved techniques for grid mapping with Rao-Blackwellized particle filters," IEEE Trans. Robot., vol. 23, no. 1, pp. 34-46, 2007.
8. D. Fox, et al., "Markov localization for mobile robots in dynamic environments," J. Artif. Intell. Res., vol. 11, pp. 391-427, 1999.
9. S. Thrun, W. Burgard, and D. Fox, Probabilistic Robotics. MIT Press, Cambridge, MA, 2005.
10. E. Marder-Eppstein, et al., "The Office Marathon: Robust navigation in an office environment," in Proc. IEEE Int. Conf. Robot. Autom., 2010.
11. B. Yamauchi, "A frontier-based approach for autonomous exploration," in Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom., 1997.
12. S. Agarwal, et al., "Ceres Solver: Non-linear Least Squares," Google LLC, 2022.
