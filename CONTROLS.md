# FRC Robot Controls Cheat Sheet

This document outlines the controls for the FRC robot, as defined in `RobotContainer.java`. The controls are context-dependent and change based on the selected controller profile.

## Shared Controls

These controls are active across all controller profiles.

### Drive Controls

| Control | Action |
| --- | --- |
| **Left Stick (Y-axis)** | Drive forward/backward. |
| **Left Stick (X-axis)** | Strafe left/right. |
| **Right Stick (X-axis)**| Rotate the robot. |

### Other Shared Controls

| Button | Action |
| --- | --- |
| **Left Bumper** | Select the left reef branch. |
| **Right Bumper** | Select the right reef branch. |
| **Left Trigger (Hold)** | Align with the reef. |
| **Right Trigger (Hold)** | Request to score. |
| **DPad Up** | Switch to **Coral Profile** and set robot goal to `SAFE_CORAL_TRANSPORT`. |
| **DPad Down** | Switch to **Algae Profile** and set robot goal to `SAFE_ALGAE_TRANSPORT`. |
| **DPad Left** | Switch to **Manual Profile** and set robot goal to `MANUAL`. |
| **DPad Right + Start Button** | Switch to **Climb Profile** and set robot goal to `MANUAL`. |
| **Right Stick + DPad Left** | Manual servo open. |
| **Right Stick + DPad Right** | Manual heading reset. |

---

## Coral Profile

| Button | Action |
| --- | --- |
| **A Button** | Request the intake coral routine (`L1`). |
| **B Button** | Request the L2 scoring routine. |
| **Y Button** | Request the L3 scoring routine. |
| **X Button** | Request the L4 scoring routine. |

---

## Algae Profile

| Button | Action |
| --- | --- |
| **A Button** | Request Score Algae Processor. |
| **B Button** | Request Intake Algae L2. |
| **Y Button** | Request Intake Algae L3. |
| **X Button** | Request Barge. |

---

## Manual Profile

**Note:** In Manual Profile, the bumpers perform an additional action *in addition* to their shared function.

| Button | Action |
| --- | --- |
| **Y Button** | Manual move elevator up. |
| **A Button** | Manual move elevator down. |
| **X Button** | Manual move diff arm out. |
| **B Button** | Manual move diff arm in. |
| **Left Bumper** | Manual rotate diff arm out (and select left reef branch). |
| **Right Bumper** | Manual rotate diff arm in (and select right reef branch). |

---

## Climb Profile

| Button | Action |
| --- | --- |
| **B Button + Start Button** | Request Abort Climb. |
| **Y Button** | Request Climb Ready. |
| **A Button** | Request Climb. |
