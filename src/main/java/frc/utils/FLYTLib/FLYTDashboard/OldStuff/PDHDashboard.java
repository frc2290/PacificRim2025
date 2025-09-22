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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.utils.FLYTLib.GlobalVar;

/** Dashboard helper for publishing REV PDH telemetry using the old FLYT API. */
public class PDHDashboard extends SuperDashboard {

  // Network tables for the controller configuration
  NetworkTable table;

  /** Power distribution hub instance being monitored. */
  PowerDistribution PDH;

  // constructor, just needs motor controller object
  public PDHDashboard(PowerDistribution PDH) {
    this.PDH = PDH;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("PowerDistributionHub");
  }

  // get PDH status
  private void PDHstate() {
    set("Voltage", PDH.getVoltage());
    set("Total Current", PDH.getTotalCurrent());
    set("Temprature", PDH.getTemperature());
    set("Channel 0", PDH.getCurrent(0));
    set("Channel 1", PDH.getCurrent(1));
    set("Channel 2", PDH.getCurrent(2));
    set("Channel 3", PDH.getCurrent(3));
    set("Channel 4", PDH.getCurrent(4));
    set("Channel 5", PDH.getCurrent(5));
    set("Channel 6", PDH.getCurrent(6));
    set("Channel 7", PDH.getCurrent(7));
    set("Channel 8", PDH.getCurrent(8));
    set("Channel 9", PDH.getCurrent(9));
    set("Channel 10", PDH.getCurrent(10));
    set("Channel 11", PDH.getCurrent(11));
    set("Channel 12", PDH.getCurrent(12));
    set("Channel 13", PDH.getCurrent(13));
    set("Channel 14", PDH.getCurrent(14));
    set("Channel 15", PDH.getCurrent(15));
    set("Channel 16", PDH.getCurrent(16));
    set("Channel 17", PDH.getCurrent(17));
    set("Channel 18", PDH.getCurrent(18));
    set("Channel 19", PDH.getCurrent(19));
    set("Channel 20", PDH.getCurrent(20));
    set("Channel 21", PDH.getCurrent(21));
    set("Channel 22", PDH.getCurrent(22));
    set("Channel 23", PDH.getCurrent(23));
  }

  // set things on the dashboard
  private void set(String name, double value) {
    table.getEntry(name).setValue(value);
  }

  @Override
  public void periodic() {
    if (GlobalVar.debug) PDHstate();
  }
}
