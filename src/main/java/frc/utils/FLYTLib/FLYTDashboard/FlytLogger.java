// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils.FLYTLib.FLYTDashboard;

import java.util.ArrayList;
import java.util.function.Supplier;

public class FlytLogger {

  private String name;

  /** Collection of all dashboard variables registered with this logger. */
  private ArrayList<FlytDashboardVariable> pubs;

  private boolean debug = false;

  /**
   * Basic Flyt Dashboard Class to create Dashboards for anything. Use: - Instantiate FlytDashboard
   * with the name of the Table you'd like to publish to. - Add publishers of various types using a
   * name and a Supplier. - Call the update() function periodically to update info. This is to be
   * called in your subsystem or other similar place.
   *
   * @param m_name - Name of the table for the dashboard
   */
  public FlytLogger(String m_name) {
    name = m_name;
    pubs = new ArrayList<FlytDashboardVariable>();
  }

  public void setDebug(boolean m_debug) {
    debug = m_debug;
  }

  /**
   * Adds a Boolean publisher to the table.
   *
   * @param m_name - Name of the variable you'd like to publish
   * @param debugOnly - Boolean to set if only update in debug mode
   * @param m_Supplier - Boolean supplier of variable. "() -> getMotorID()" for example.
   */
  public void addBoolPublisher(String m_name, boolean debugOnly, Supplier<Boolean> m_Supplier) {
    FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name, debugOnly);
    temp.addBooleanSupplier(m_Supplier);
    pubs.add(temp);
  }

  /**
   * Adds a Integer publisher to the table.
   *
   * @param m_name - Name of the variable you'd like to publish
   * @param debugOnly - Boolean to set if only update in debug mode
   * @param m_Supplier - Integer supplier of variable. "() -> getMotorID()" for example.
   */
  public void addIntegerPublisher(String m_name, boolean debugOnly, Supplier<Integer> m_Supplier) {
    FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name, debugOnly);
    temp.addIntegerSupplier(m_Supplier);
    pubs.add(temp);
  }

  /**
   * Adds a Double publisher to the table.
   *
   * @param m_name - Name of the variable you'd like to publish
   * @param debugOnly - Boolean to set if only update in debug mode
   * @param m_Supplier - Double supplier of variable. "() -> getMotorID()" for example.
   */
  public void addDoublePublisher(String m_name, boolean debugOnly, Supplier<Double> m_Supplier) {
    FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name, debugOnly);
    temp.addDoubleSupplier(m_Supplier);
    pubs.add(temp);
  }

  /**
   * Adds a String publisher to the table.
   *
   * @param m_name - Name of the variable you'd like to publish
   * @param debugOnly - Boolean to set if only update in debug mode
   * @param m_Supplier - String supplier of variable. "() -> getMotorID()" for example.
   */
  public void addStringPublisher(String m_name, boolean debugOnly, Supplier<String> m_Supplier) {
    FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name, debugOnly);
    temp.addStringSupplier(m_Supplier);
    pubs.add(temp);
  }

  /**
   * Update function to publish updates on every variable added to table. Meant to be called in
   * periodic of subsystem its implemented in.
   */
  public void update(boolean m_debug) {
    debug = m_debug;
    for (FlytDashboardVariable pub : pubs) {
      pub.update(debug);
    }
  }

  public double getDouble(String m_name) {
    for (FlytDashboardVariable item : pubs) {
      if (item.getName().equals("/" + name + "/" + m_name) && (!item.debugOnly() || debug)) {
        return item.getDouble();
      }
    }
    return 0.0;
  }
}
