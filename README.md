# Daniel's Experimental GRT 2026 Robot Code!

## Cool Things added:

### Cool files to checkout:
- `src/main/java/frc/robot/util/LoggedCanivore.java` — `CANBus` wrapper that polls bus status on a background thread and auto-logs utilization / bus-off / TEC / REC through AdvantageKit.
- `src/main/java/frc/robot/util/PhoenixUtil.java` — `tryUntilOk` config-retry helper plus a per-bus status-signal registry so every CAN bus refreshes its signals together.
- `src/main/java/frc/robot/subsystems/hopper/HopperIO.java` — AdvantageKit `@AutoLog` IO interface for the hopper (position, velocity, currents, temp, faults) with `setDutyCycle` / `setVelocity` / `stop`.
- `src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java` — TalonFX implementation of `HopperIO`: stator current limits, Slot 0 velocity PID, and signal registration into `PhoenixUtil`.

### TODO:

- [ ] Fix code style inconsistencies/overall weirdness
    - [x] Add `checkstyle` code style checking
        - [ ] Create checkstyle docs
    - [x] Fix things caught by `checkstyle`
    - [ ] Consistent command usage (inline commands in robot container and triggers and stuff)
- [ ] Full advantage kit port
- [ ] Figure out sim stuff

### Change Log:
1. Generated checkstyle config found in `./config/dev/checkstyle_config.xml` by running the google checkstyle config on current code and making changes based on preferred outcome
2. Added `LoggedCanivore` (`src/main/java/frc/robot/util/LoggedCanivore.java`) — wraps `CANBus` with a background thread that polls bus status every 400ms and auto-logs utilization, bus-off/TEC/REC counters through AdvantageKit. Idea borrowed from FRC 6328.
3. Added `PhoenixUtil` (`src/main/java/frc/robot/util/PhoenixUtil.java`, from FRC 6328) — provides `tryUntilOk` for retrying Phoenix config calls and a per-bus `registerSignals` / `refreshAllStatusSignals` registry so every CAN bus refreshes its signals together.
4. Replaced the `SWERVE_CAN_BUS` / `MECH_CAN_BUS` string constants with a `Constants.CANType` enum (`RIO`, `MECH`, `SWERVE`) so bus identity is type-safe and usable as a map key in `PhoenixUtil`.

## Robot Control and Motor Assignments

### **Drive Controller (Pilot)**

| PS5 Button             | Assigned Function                                       | Motor ID |
| ---------------------- | ------------------------------------------------------- | -------- |
| X (Cross)              | Reset Heading                                           |          |
| O (Circle)             |                                                         |          |
| △ (Triangle)           | Auto Climb (TODO)                                       |          |
| □ (Square) (hold)      | Emergency Force Intake In (pivot up + stop rollers)     |          |
| L1 (hold)              | Robot-Relative Mode (release returns to field-relative) |          |
| L2 (analog)            | Speed Limit (harder press = slower)                     |          |
| R1 (toggle)            | Shooter Sequence                                        |          |
| R2 (>= 90% pressed)    | Boost Mode                                              |          |
| L3 (Left Stick Click)  |                                                         |          |
| R3 (Right Stick Click) |                                                         |          |
| D-Pad Up               |                                                         |          |
| D-Pad Down             |                                                         |          |
| D-Pad Left             |                                                         |          |
| D-Pad Right            |                                                         |          |
| Options Button         | Reset Pose to Starting Position                         |          |
| Create Button          |                                                         |          |
| PS Button              |                                                         |          |
| Touchpad               |                                                         |          |
| Left Joystick          | Swerve Translation (forward/strafe)                     | 0-11     |
| Right Joystick         | Swerve Rotation                                         | 0-11     |

### **Mech Controller (Operator)**

| PS5 Button             | Assigned Function                        | Motor ID |
| ---------------------- | ---------------------------------------- | -------- |
| X (Cross)              | Forced Pivot In (HOLD)                   |          |
| O (Circle)             | Pivot In / Out (Click for IN/OUT)        |          |
| △ (Triangle)           | Shooter Hood Positions (Multiple Clicks) |          |
| □ (Square)             | Shooter Wheel (Click for ON/OFF)         |          |
| L1                     | Hopper Shoot All Balls (HOLD)            |          |
| L2                     | Intake Rollers In Manual                 |          |
| R1                     | Hopper Reverse (Agitator) (HOLD)         |          |
| R2                     | Intake Rollers Out Manual                |          |
| L3 (Left Stick Click)  |                                          |          |
| R3 (Right Stick Click) |                                          |          |
| D-Pad Up               | Winch up Manual                          |          |
| D-Pad Down             | Winch down Manual                        |          |
| D-Pad Left             | Doornob down Manual                      |          |
| D-Pad Right            | Doornob up Manual                        |          |
| Options Button         |                                          |          |
| Create Button          |                                          |          |
| PS Button              |                                          |          |
| Touchpad               |                                          |          |
| Left Joystick          | Hood Up / Down Manual                    |          |
| Right Joystick         | Intake Pivot Up / Down Manual            |          |

## **CAN Assignments**

| Subsystem       | Component       | Type        | Position | CAN ID       |
| --------------- | --------------- | ----------- | -------- | ------------ |
| **Swerve**      | Drive           | Kraken x60  | FL       | 1            |
|                 | Drive           | Kraken x60  | FR       | 3            |
|                 | Drive           | Kraken x60  | BR       | 5            |
|                 | Drive           | Kraken x60  | BL       | 7            |
|                 | Steer           | Kraken x44  | FL       | 0            |
|                 | Steer           | Kraken x44  | FR       | 2            |
|                 | Steer           | Kraken x44  | BL       | 4            |
|                 | Steer           | Kraken x44  | BR       | 6            |
|                 | CANcoder        | CANcoder    | FL       | 11           |
|                 | CANcoder        | CANcoder    | FR       | 10           |
|                 | CANcoder        | CANcoder    | BL       | 8            |
|                 | CANcoder        | CANcoder    | BR       | 9            |
| **Intake**      | Pivot           | Kraken x60  | -        | 12           |
|                 | Pivot Encoder   | Throughbore | -        | 13           |
|                 | Roller          | Kraken x44  | -        | 14           |
| **Hopper**      | Hopper          | Kraken x44  | -        | 15           |
| **Shooter**     | Hood            | Kraken x44  | -        | 16           |
|                 | Flywheel        | Kraken x44  | -        | 17           |
|                 | Hood Encoder    | Throughbore | -        | 18           |
| **Climb**       | Doornob         | Kraken x44  | -        | 19           |
|                 | Doornob Encoder | Throughbore | -        | 20           |
|                 | Winch           | Kraken x60  | -        | 21           |
| **CAN Devices** | CANdi           | CANdi       | -        | 22           |
|                 | CANdi           | CANdi       | -        | 23           |
|                 | CANivore        | CANivore    | -        | "swerve-can" |
|                 | CANivore        | CANivore    | -        | "mech-can"   |
|                 | Pigeon          | Pigeon 2.0  | -        | 24           |
| **Extra**       | Second Flywheel | X60         | -        | 25           |
|                 | Tower Roller    | X40         | -        | 26           |
|                 | CANrange climb  |             | -        | 22           |

### **Motor Configuration**

| Subsystem   | Motor Name | ID  | Current Limit | Max Velocity | Max Current | Max Acceleration |
| ----------- | ---------- | --- | ------------- | ------------ | ----------- | ---------------- |
| **Swerve**  | Drive FL   | 1   |               |              |             |                  |
|             | Drive FR   | 3   |               |              |             |                  |
|             | Drive BR   | 5   |               |              |             |                  |
|             | Drive BL   | 7   |               |              |             |                  |
|             | Steer FL   | 0   |               |              |             |                  |
|             | Steer FR   | 2   |               |              |             |                  |
|             | Steer BL   | 4   |               |              |             |                  |
|             | Steer BR   | 6   |               |              |             |                  |
| **Intake**  | Pivot      | 12  |               |              |             |                  |
|             | Roller     | 14  |               |              |             |                  |
| **Hopper**  | Hopper     | 15  |               |              |             |                  |
| **Shooter** | Hood       | 16  |               |              |             |                  |
|             | Flywheel   | 17  |               |              |             |                  |
| **Climb**   | Doornob    | 19  |               |              |             |                  |
|             | Winch      | 21  |               |              |             |                  |

Mech Controller Keybinds:

Button Action
R1 Intake In
R2 Intake Out
L1 Hopper Forward
L2 Hopper Reverse
Right Stick Y Pivot Manual
Left Stick Y Hood/Arm Manual
Square + R2 Flywheel Speed
Cross (hold) Climb Down
Triangle (hold) Climb Up
D-Pad Up/Down Winch Up/Down
