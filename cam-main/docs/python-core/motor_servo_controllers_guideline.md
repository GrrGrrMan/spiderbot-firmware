# SpiderBot Motor and Servo Control Documentary Guideline

This document provides a comprehensive technical overview and operational guideline of the motor, servo, and LED controller systems implemented on the SpiderBot hexapod platform. It details physical safety practices, communication architectures, command protocol schemas (V1, V2, and V3), inverse kinematics integration, and comparison of available control scripts.

---

## 1. System Architecture & Communication Flow

Control commands for SpiderBot follow a structured, multi-layer journey from high-level user interfaces down to physical joint movements:

```text
[ High-Level UI / Python Script ]
               │
               ├─► MQTT Broker (Port 1883) ──► [ ESP32-S3 / CAM Nodes ]
               │                                       │
               └─► Direct TCP Socket (Port 7777) ─────┤
                                                       ▼
                                            [ PCA9685 16-Ch PWM Driver ]
                                                       │
                                                       ▼
                                            [ Servos & Status LEDs ]
```

### 1.1 Key Hardware Components
- **Raspberry Pi Hub**: Serves as the central control station, hosting a Mosquitto MQTT broker, Tailscale network node, and link scanning watchdog services.
- **ESP32 Microcontrollers (ESP32-S3 / ESP32-CAM)**: Receive and parse control commands from MQTT or direct TCP sockets, executing movement sequences on local hardware timers.
- **PCA9685 16-Channel PWM Driver**: Communicates with the ESP32 via I2C. Generates high-resolution PWM signals to actuate hobby servos (channels 0–12) and drive multi-channel status LEDs (channels 13 and 14).
- **Hobby Servos**: Core joint actuators for the hexapod's legs. Standard operating angle is **0° to 180°**, mapped to equivalent PWM duty cycles.

### 1.2 Configuration & Topic Roots
To support multiple hardware configurations dynamically, the system relies on [**`mqtt_common.py`**](Hexapod-Main/Scripts/functions/mqtt_common.py) to manage network targets and load MQTT settings.
- **Rules**: Never hardcode topic roots. Always derive them at runtime from `settings.topic_root` in [`mqtt_common.py`](Hexapod-Main/Scripts/functions/mqtt_common.py).
- **Default Roots**:
  - `esp32s3`: `alphaesp32s3/spiderbot-s3`
  - `esp32cam`: `alphaesp32/spiderbot-cam`

---

## 2. Physical Safety and Power Management

Hobby servos utilize holding torque to stay in their commanded positions. Safely interacting with the hardware is critical to prevent electronic brownouts and physical linkage damage.

### 2.1 The Brownout Hazard (Physical Forcing)
When a servo is actively powered and holding its position:
- **Resistive Load**: Physically turning a joint causes the servo's internal motor to fight back to correct the position.
- **Current Draw**: This resistance can draw in excess of **1.0 Ampere** per servo.
- **ESP32 Brownout**: Multi-servo resistance spikes can easily overwhelm the ESP32's 3.3V power regulator, causing immediate system restarts, software crashes, or local broker disconnection.

### 2.2 Safe Handling Protocol
Before touching or adjusting any physical joint by hand:
1. Send a **Free** command to the affected channel or the entire board.
2. Verify that holding torque is eliminated (the joints should move passively with ease).
3. Physically position the robot.
4. Send a **Center** or **Set** command to re-engage the PWM holding signal.

### 2.3 `Free` vs. `Detach` Semantics
Understanding the programmatic distinction between freeing and detaching is critical:
- `tester:servo:free:<pin>` / `servo:free` (Free): Removes hold torque by disabling active duty cycle output on the PCA9685 channel, but keeps the servo logic attached. Use this when you want passive movement but intend to re-engage the same servo soon.
- `tester:servo:detach:<pin>` (Detach): Completely disconnects the PWM signal and releases the hardware timer. Best for long-term power saving when a joint or accessory will not be used for an extended period.

---

## 3. Command Topic Routing

The ESP32 firmware exposes three distinct command topics to prevent processing overhead and message collision. Sending a command to the incorrect topic will cause the firmware to **silently drop** the command without an error response.

| Topic Path | Topic Code Constant | Purpose |
|------------|---------------------|---------|
| `{root}/cmd/discrete` | `TOPIC_CMD` | Discrete/one-off actions: lights, single LEDs, manual servo adjustments, tester probes, and ESP-level commands (OTA, resets). |
| `{root}/cmd/motion` | `TOPIC_MOTION` | High-frequency continuous locomotion, walking gait coordinate sets, and real-time IK target profiles. |
| `{root}/cmd/motor` | `TOPIC_MOTOR` | Dedicated low-latency channel for sending Motor V2 program scripts without the `motor:` prefix. |

---

## 4. Standard Servo Control (V1 / Legacy)

Standard servo commands are sent to the `{root}/cmd/discrete` topic. They are designed for individual joint manual control, calibration, and hardware test rigs.

### 4.1 Basic Commands
- **Manual Move**: `servo:<ch>:<angle>[:<ms>]`
  - Moves the specified channel `ch` (0–15) to a target `angle` (0–180) over optional duration `ms`.
  - Example: `servo:2:120:750` *(Moves channel 2 to 120 degrees over 750 milliseconds)*
- **Freewheel Channel**: `servo:free[:<ch>]`
  - Releases holding torque on a single channel or all channels if `ch` is omitted.
- **Center Channel**: `servo:center[:<ch>]`
  - Moves a single channel (or all channels if omitted) to its default calibration midpoint (**90°**).
- **Status Query**: `servo:status`
  - Prompts the ESP to publish a snapshot of current channel angle states.

### 4.2 Direct GPIO Servo Tester Commands (`tester:servo:`)
For accessories or hardware tests outside the primary leg configurations, the `tester:servo:` commands bypass high-level leg models and communicate directly with raw board GPIOs:
- **Attach**: `tester:servo:attach:<pin>` *(Registers a GPIO pin as an active servo channel)*
- **Set Angle**: `tester:servo:set:<pin>:<angle>` *(Sets the target angle)*
- **Set Microseconds**: `tester:servo:us:<pin>:<pulse_us>` *(Sets raw pulse width in microseconds, typically 500–2500us)*
- **Free**: `tester:servo:free:<pin|all>` *(Suspends PWM hold on a tester pin)*
- **Detach**: `tester:servo:detach:<pin|all>` *(Unregisters the tester pin completely to release memory/timers)*
- **Status**: `tester:servo:status` *(Lists all currently active tester pin assignments and parameters)*

---

## 5. Programmatic RAM Sequence Layer (Motor V2)

To avoid network congestion from sending continuous high-frequency angle frames over Wi-Fi, the **Motor V2** protocol allows scripts to be defined, stored in ESP RAM, and executed locally on the microcontroller. 

> ⚠️ **Memory Warning**: Programs are stored in the ESP32's volatile RAM. Any system reboot, OTA update, or power cycle will **permanently clear** all loaded programs.

### 5.1 Program Lifetime (Load, Add, Run, Stop)
Programs can be built step-by-step or submitted as a single multi-command payload:

#### Method A: Multi-Command Payload (One-liner)
Separate commands with a semicolon on a single MQTT publish:
```text
motor:load sweep loops=0; pose 1000 0=90 1=90; pose 1000 0=180 1=180; run
```

#### Method B: Iterative Piece-by-Piece Assembly
Useful for building longer, complex sequences dynamically:
```text
motor:load nod loops=3
motor:add nod pose 500 0=70 1=110
motor:add nod pose 500 0=110 1=70
motor:run nod
```

#### HALT Execution
- `motor:stop` halts the running script immediately, keeping all servos holding their current position.
- `motor:stop free` halts the running script immediately and cuts holding torque (PWM) to all servos.

### 5.2 Direct V2 Commands
You can also run immediate V2 actions without compiling them into a script sequence:
- `motor:move <ch> <angle> <duration_ms>` *(Move single channel with controlled duration)*
- `motor:pose <duration_ms> <ch>=<angle> [<ch>=<angle> ...]` *(Simultaneous multi-channel pose)*
- `motor:center all <duration_ms>` *(Smoothly centers all channels)*
- `motor:free all` *(Instantly freewheels all channels)*

### 5.3 Stagger Penalty (`CFG_MOTOR_V2_POSE_CHANNEL_STAGGER_MS`)
When a multi-channel `pose` command is executed, the ESP firmware deliberately staggers writing to each channel to prevent massive instantaneous power draw spikes and current ripples on the power rail.
- **Default Stagger Gap**: **8 ms** per channel.
- **Mathematical Delay**: For $N$ channels, the actual movement takes:
  $$\text{Completion Time} = \text{duration\_ms} + (N - 1) \times 8\text{ ms}$$
- **Operational Pitfall**: A pose targeting 6 leg joints over 1000 ms will actually finish in $1000 + 5 \times 8 = 1040\text{ ms}$. This delay must be accounted for in tight synchronization loops.

### 5.4 Safety and Parameter Constraints
The Motor V2 controller enforces strict safety guardrails in software and firmware:
- **Hard Angle Rejections**: Unlike V1, which might clamp values, the V2 firmware **rejects** out-of-range commands with an explicit `ERR` response if any target angle is outside `0..180`.
- **System Constraints**:
  - `MAX_PROGRAM_NAME_LEN`: 15 characters (A-Z, 0-9, `_`, `-` only).
  - `MAX_PROGRAM_STEPS`: 24 steps (V2) or 50 steps (V3).
  - `MAX_LOOPS`: 10,000 loops (`loops=0` designates infinite looping).
  - `MAX_MOVE_MS`: 120,000 ms (2 minutes maximum travel time).
  - `MAX_WAIT_MS`: 300,000 ms (5 minutes maximum wait step).

---

## 6. Inverse Kinematics & Multi-DOF Interpolation (Motor V3)

The advanced controller layer in [**`leg_kinematics.py`**](Hexapod-Main/Scripts/leg/leg_kinematics.py) (sometimes referred to as **Servo Controller V3**) integrates a complete 3-Degree-of-Freedom (3DOF) Inverse Kinematics (IK) engine, per-leg mounting offsets, stagger corrections, and non-linear movement easing.

### 6.1 The 3DOF IK Engine
The function [`_ik_3dof_leg_coxa_femur_tibia()`](Hexapod-Main/Scripts/leg/leg_kinematics.py:143) maps target coordinates in the leg's local frame $(X, Y, Z)$ into physical joint angles (Coxa yaw, Femur pitch, Tibia pitch).

- **Coordinate System**:
  - $X$ = Lateral axis (outward from leg root, in mm).
  - $Y$ = Forward axis (parallel to leg root, in mm).
  - $Z$ = Up/Down axis (vertical height, in mm).

#### Mathematical Principles
1. **Coxa Yaw**: Calculated directly using the horizontal axes:
   $$\text{coxa\_yaw} = \text{atan2}(y, x)$$
2. **2D Linkage (Femur-Tibia Planar IK)**:
   Computes the effective planar reach from the femur root:
   $$r_{\text{eff}} = \sqrt{x^2 + y^2} - l_{\text{coxa}}$$
   Using the Cosine Rule, the joint angles for Femur pitch ($\alpha$) and Tibia pitch ($\beta$) are solved to position the foot tip at $(r_{\text{eff}}, z)$.

### 6.2 Reachability & Safety Validation
To prevent physical crashes, motor strain, or unresolvable mathematical conditions (e.g., trying to divide by zero when taking an inverse cosine of a number $>1$), the engine performs strict physical reach boundaries checks:
- **Min Reach Boundary**: $d_{\text{min}} = |l_{\text{femur}} - l_{\text{tibia}}| + 1\text{ mm}$
- **Max Reach Boundary**: $d_{\text{max}} = l_{\text{femur}} + l_{\text{tibia}} - 1\text{ mm}$
- **Rejection**: If target distance $d = \sqrt{r_{\text{eff}}^2 + z^2}$ is outside $[d_{\text{min}}, d_{\text{max}}]$, a `ValueError` is raised, and command execution is aborted safely before any packet is transmitted to the robot.

### 6.3 Per-Leg Configuration Calibration
Legs on the hexapod are mounted in different positions and orientations. This spatial arrangement is resolved via the [`LEG_CONFIG`](Hexapod-Main/Scripts/leg/leg_kinematics.py:99) mapping:

```python
LEG_CONFIG = {
    1: {"ch": (0, 1, 2), "dir": (1.0, 1.0, 1.0), "off": (90.0, 90.0, 90.0), "mount_deg": 45.0},
    2: {"ch": (3, 4, 5), "dir": (-1.0, 1.0, 1.0), "off": (90.0, 90.0, 90.0), "mount_deg": 0.0},
    3: {"ch": (6, 7, 8), "dir": (1.0, 1.0, 1.0), "off": (90.0, 90.0, 90.0), "mount_deg": -45.0},
    4: {"ch": (9, 10, 11), "dir": (-1.0, 1.0, 1.0), "off": (90.0, 90.0, 90.0), "mount_deg": 135.0},
}
```

- **`ch`**: Links local joints (Coxa, Femur, Tibia) to PCA9685 physical PWM output channels.
- **`dir`**: Configures joint direction multipliers. Values of `-1.0` dynamically invert joint rotations, allowing the same IK coordinate frames to be used on mirrored sides of the hexapod body.
- **`off`**: Calibration offsets (neutral calibration angles, defaulting to **90.0°**).
- **`mount_deg`**: The physical mounting angle of the leg root relative to the robot's main center longitudinal axis.

### 6.4 Stagger Compensation
Because of the 8 ms write gap per active channel in the firmware, joint coordinates targeting multiple legs would arrive with a cumulative delay, causing some legs to finish their stroke noticeably late.

To compensate for this, [`leg_kinematics.py`](Hexapod-Main/Scripts/leg/leg_kinematics.py:1255) implements **Stagger Deduction**:
```python
stagger_penalty = max(0, len(target_dict) - 1) * 8
effective_duration = max(0, total_duration - stagger_penalty)
```
The client-side generator subtracts the stagger penalty from the target program step duration, ensuring the motion completes exactly when the user intended.

### 6.5 Non-Linear Easing & Interpolation
When planning paths, executing linear steps can cause sudden jerky joint accelerations. The V3 Kinematics layer offers both linear and eased trajectory expansion modes:

1. **Linear Interpolation**:
   Sends a single group `pose` command with `effective_duration`. Joint movement velocity is uniform.
2. **Non-Linear Interpolation (Easing)**:
   Splits the movement path into multiple micro-steps (configured by `step_ms`). For each step, it calculates the progress variable $t$ ($0.0 \to 1.0$) and applies mathematical easing:
   - **Ease In** ($t^2$): Slow start, fast end.
   - **Ease Out** ($1 - (1 - t)^2$): Fast start, slow end.
   - **Ease In-Out**: Slow start, fast middle, slow end (sinusoidal-like curve).
     $$f(t) = \begin{cases} 2t^2 & \text{if } t < 0.5 \\ 1 - \frac{(-2t + 2)^2}{2} & \text{otherwise} \end{cases}$$

The generator interpolates the joint angles at each micro-step and queues them as successive V2 steps, resulting in organic, smooth joint trajectories.

---

## 7. Comparative Reference of Controller Scripts

The SpiderBot codebase provides several scripts for motor control, each optimized for different stages of development and operation:

### 7.1 [`Servo_controller.py`](Hexapod-Main/Scripts/general%20(Old)/Servo_controller.py)
- **Primary Mode**: Manual slider/button controls using standard V1 commands.
- **Key Feature**: Contains a thread-safe `_CommandQueue` that throttles sequential MQTT publications with a configurable delay (default **70 ms**). This prevents flooding the ESP32's limited **1 kB** PubSubClient MQTT buffer and crashing the board.
- **Best For**: Manual calibration, individual joint testing, and direct physical servo testing.

### 7.2 [`Servo_controller_v2.py`](Hexapod-Main/Scripts/general%20(Old)/Servo_controller_v2.py)
- **Primary Mode**: Programmatic RAM-sequence builder targeting Motor V2.
- **Key Feature**: Allows local compilation of structured program step objects ([`ProgramStep`](Hexapod-Main/Scripts/leg/leg_kinematics.py:202)), then publishes them to the local ESP memory. Bypasses the need for real-time network streams, eliminating packet loss jitter.
- **Best For**: Simple looping physical actions (e.g., nodding, sweeping, tail-wagging) and network-efficient sequence playback.

### 7.3 [`leg_kinematics.py`](Hexapod-Main/Scripts/leg/leg_kinematics.py)
- **Primary Mode**: 3DOF Inverse Kinematics, trajectory interpolation, and calibration manager.
- **Key Feature**: Fully implements coordinate-based positioning $(X, Y, Z)$ for legs 1–4, and handles automatic non-linear step easing curves and stagger penalties.
- **Best For**: Advanced hexapod step sequences, coordinate-based pose styling, and leg calibration tuning.

### 7.4 [`local_esp_control.py`](Hexapod-Main/Scripts/functions/local_esp_control.py)
- **Primary Mode**: Low-level TCP CLI client connecting directly to the ESP control port (**7777**).
- **Key Feature**: Bypasses the MQTT broker completely. Connects to the ESP over raw sockets and exchanges command/status messages with a signed cryptographic linking flow.
- **Best For**: Direct terminal-based diagnostic testing, networking debug, and offline/standalone hardware operations.
