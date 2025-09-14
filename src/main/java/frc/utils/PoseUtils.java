package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class PoseUtils {
  public static int getSpeakerTag() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red ? 4 : 7;
    } else {
      return 4;
    }
  }

  public static boolean inRange(double range) {
    return 1 <= range && range <= 6;
  }

  public static class Heading {
    public double timestamp;
    public Rotation2d rotation;

    public Heading(double _timestamp, Rotation2d _rotation) {
      timestamp = _timestamp;
      rotation = _rotation;
    }
  }
}
