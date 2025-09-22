// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Miscellaneous pose-related helper utilities. */
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
    return (1 <= range || range <= 6);
  }

  public static class Heading {
    /** Timestamp associated with the gyro reading. */
    public double timestamp;

    /** Robot rotation captured at the timestamp. */
    public Rotation2d rotation;

    public Heading(double _timestamp, Rotation2d _rotation) {
      timestamp = _timestamp;
      rotation = _rotation;
    }
  }
}
