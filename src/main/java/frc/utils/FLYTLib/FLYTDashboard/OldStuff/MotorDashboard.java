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
package frc.utils.FLYTLib.FLYTDashboard.OldStuff;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
import frc.utils.FLYTLib.GlobalVar;

/** Dashboard helper that exposes PID tuning fields for FLYT motor controllers. */
public class MotorDashboard extends SuperDashboard {

  // Network tables for the controller configuration
  NetworkTable table;

  /** Motor controller being tuned by this dashboard view. */
  FlytMotorController controller;

  private NetworkTableEntry kP, kI, kD, kFF, set;

  // constructor, just needs motor controller object
  public MotorDashboard(FlytMotorController m_controller) {
    controller = m_controller;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Motor" + String.valueOf(controller.getMotorID()));
    kP = table.getEntry("kP");
    kI = table.getEntry("kI");
    kD = table.getEntry("kD");
    kFF = table.getEntry("kFF");
    set = table.getEntry("SetPoint");
    kP.setDouble(0);
    kI.setDouble(0);
    kD.setDouble(0);
    kFF.setDouble(0);
    set.setDouble(0);
    controller.pidSetup(-1, 1, 0, 0, true, 0);
  }

  public void test() {
    System.out.println("Howdy");
  }

  // works on printing motor status
  private void motorState() {
    set("MotorID", controller.getMotorID());
    set("MotorPosition", controller.getPos());
    set("MotorVelocity", controller.getVel());
    set("MotorVoltage", controller.getVol());
    set("MotorCurrent", controller.getCurrent());
    set("MotorTempreture", controller.getTemp());
  }

  // tunes the motor gains
  private void motorTune(double kp, double ki, double kd, double ff) {
    controller.pidTune(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0), kFF.getDouble(0));
  }

  // set things on the dashboard
  private void set(String name, double value) {
    table.getEntry(name).setValue(value);
  }

  @Override
  public void periodic() {
    if (GlobalVar.debug) {
      motorState();
    }
    if (GlobalVar.debug) {
      motorTune(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0), kFF.getDouble(0.0));
      controller.set(set.getDouble(0.0));
    }
  }
}
