package frc.robot.FLYTLib.FLYTDashboard;

import java.util.ArrayList;
import java.util.function.Supplier;

public class FlytLogger {

    private String name;
    private ArrayList<FlytDashboardVariable> pubs;

    /**
     * Basic Flyt Dashboard Class to create Dashboards for anything.
     * Use:
     * - Instantiate FlytDashboard with the name of the Table you'd like to publish to.
     * - Add publishers of various types using a name and a Supplier.
     * - Call the update() function periodically to update info. This is to be called in your subsystem or other similar place.
     * 
     * @param m_name - Name of the table for the dashboard
     */
    public FlytLogger(String m_name) {
        name = m_name;
        pubs = new ArrayList<FlytDashboardVariable>();
    }

    /**
     * Adds a Boolean publisher to the table.
     * 
     * @param m_name - Name of the variable you'd like to publish
     * @param m_Supplier - Boolean supplier of variable. "() -> getMotorID()" for example.
     */
    public void addBoolPublisher(String m_name, Supplier<Boolean> m_Supplier) {
        FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name);
        temp.addBooleanSupplier(m_Supplier);
        pubs.add(temp);
    }

    /**
     * Adds a Integer publisher to the table.
     * 
     * @param m_name - Name of the variable you'd like to publish
     * @param m_Supplier - Integer supplier of variable. "() -> getMotorID()" for example.
     */
    public void addIntegerPublisher(String m_name, Supplier<Integer> m_Supplier) {
        FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name);
        temp.addIntegerSupplier(m_Supplier);
        pubs.add(temp);
    }

    /**
     * Adds a Double publisher to the table.
     * 
     * @param m_name - Name of the variable you'd like to publish
     * @param m_Supplier - Double supplier of variable. "() -> getMotorID()" for example.
     */
    public void addDoublePublisher(String m_name, Supplier<Double> m_Supplier) {
        FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name);
        temp.addDoubleSupplier(m_Supplier);
        pubs.add(temp);
    }

    /**
     * Adds a String publisher to the table.
     * 
     * @param m_name - Name of the variable you'd like to publish
     * @param m_Supplier - String supplier of variable. "() -> getMotorID()" for example.
     */
    public void addStringPublisher(String m_name, Supplier<String> m_Supplier) {
        FlytDashboardVariable temp = new FlytDashboardVariable("/" + name + "/" + m_name);
        temp.addStringSupplier(m_Supplier);
        pubs.add(temp);
    }

    /**
     * Update function to publish updates on every variable added to table.
     * Meant to be called in periodic of subsystem its implemented in.
     */
    public void update() {
        for (FlytDashboardVariable pub : pubs) {
            pub.update();
        }
    }

    public double getDouble(String m_name) {
        for (FlytDashboardVariable item : pubs) {
            if (item.getName().equals("/" + name + "/" + m_name)) {
                return item.getDouble();
            }
        }
        return 0.0;
    }
}
