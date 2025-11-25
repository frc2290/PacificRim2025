# PhotonVision Guide

## How `PhotonRunnable` Works

`src/main/java/frc/utils/PhotonRunnable.java` wraps the PhotonVision functionality into a separate thread to avoid blocking the main robot loop. Here is a breakdown of its operation:

1.  **Initialization**:
    *   It initializes a `PhotonCamera` with a specific camera name.
    *   It loads the AprilTag field layout (currently set to `k2025ReefscapeAndyMark`).
    *   It sets up a `PhotonPoseEstimator` which uses the `CONSTRAINED_SOLVEPNP` strategy to estimate the robot's position based on visible tags.

2.  **Execution (`run()` method)**:
    *   This method is called periodically (every 10ms as configured in `PoseEstimatorSubsystem`).
    *   It fetches all unread results from the `PhotonCamera`.
    *   For each result:
        *   It injects the current gyro heading into the pose estimator to improve accuracy.
        *   It updates the pose estimator with the vision result.
        *   If a pose is calculated, it filters it based on distance (must be < 3 meters) and ambiguity (must be < 0.2).
        *   Valid poses are stored in an `AtomicReference` (`atomicEstimatedRobotPose`), ensuring thread safety.

3.  **Data Retrieval**:
    *   The `grabLatestEstimatedPose()` method atomically retrieves and clears the latest pose. This ensures that `PoseEstimatorSubsystem` consumes each estimate only once.

## How to Add a 3rd Camera

To add a third camera, you need to modify `Constants.java` and `PoseEstimatorSubsystem.java`.

### 1. Define Camera Constants
In `src/main/java/frc/robot/Constants.java`, under `VisionConstants`, add the transformation for the new camera relative to the robot center.

```java
public static final Transform3d APRILTAG_CAMERA3_TO_ROBOT =
    new Transform3d(
        new Translation3d(x, y, z), // Replace with actual values
        new Rotation3d(roll, pitch, yaw)); // Replace with actual values
```

### 2. Update `PoseEstimatorSubsystem`
In `src/main/java/frc/utils/PoseEstimatorSubsystem.java`:

**A. Declare the new components:**

```java
private final PhotonRunnable photonEstimator3;
private final Notifier photonNotifier3;
```

**B. Initialize them in the constructor:**

```java
photonEstimator3 =
    new PhotonRunnable(
        "Camera3Name", // Replace with your camera name
        VisionConstants.APRILTAG_CAMERA3_TO_ROBOT,
        () -> getHeading());

photonNotifier3 = new Notifier(photonEstimator3);
```

**C. Start the thread:**

```java
photonNotifier3.setName("PhotonRunnable3");
photonNotifier3.startPeriodic(0.01);
```

**D. Consume data in `periodic()`:**

```java
var visionPose3 = photonEstimator3.grabLatestEstimatedPose();
if (visionPose3 != null) {
  sawTag = true;
  var pose2d3 = visionPose3.estimatedPose.toPose2d();
  if (originPosition != kBlueAllianceWallRightSide) {
    pose2d3 = flipAlliance(pose2d3);
  }
  poseEstimator.addVisionMeasurement(pose2d3, visionPose3.timestampSeconds);
}
```
