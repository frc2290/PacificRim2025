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
package frc.utils.FLYTLib;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.FLYTLib.FLYTDashboard.OldStuff.PDHDashboard;

/** Legacy subsystem that exposes the NavX and PDH over NetworkTables. */
public class RobotSystem extends SubsystemBase {

  // set naviax2mxp gyro on spi
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  // Rev power distribution panel
  private PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev); // set it up

  /** Dashboard view that publishes PDH telemetry over NetworkTables. */
  private PDHDashboard PDHDash = new PDHDashboard(PDH); // initialize the dashboard

  // Network tables for the controller configuration
  NetworkTable table;
  private NetworkTableEntry kGyroOffest;

  private double gyro_yawOffset = 0.0; // set gyro offset

  public RobotSystem() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Gyro");
    kGyroOffest = table.getEntry("GyroOffset");
    // set up the gyro
    gyro.reset();
  }

  /**
   * Returns the current yaw angle of the gyro
   *
   * @return
   */
  public double gyro_getYaw() {
    return (gyro.getYaw() + gyro_yawOffset + 360) % 360;
  }

  /**
   * Takes in the angle offset to compensate gyro drifft or offset
   *
   * @param angle
   */
  public void gyro_setYaw(double angle) {
    gyro_yawOffset = angle - gyro.getYaw();
  }

  @Override
  public void periodic() {

    if (GlobalVar.debug) {
      // update the gyro offset
      gyro_setYaw(kGyroOffest.getDouble(0.0));

      // update the network tables
      table.getEntry("GyroYaw").setDouble(gyro.getYaw());
      table.getEntry("GyroYawOffset").setDouble(gyro_yawOffset);
    }
  }
}
