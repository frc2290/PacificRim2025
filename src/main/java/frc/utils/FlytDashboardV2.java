package frc.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;

/**
 * A comprehensive manager for robot-dashboard communication using the modern
 * NetworkTables PubSub API.
 * <p>
 * This class provides a centralized way to publish robot data to the dashboard
 * and subscribe to values sent from the dashboard, all within a single,
 * clean API. It is designed to be instantiated once per robot, and its methods
 * can be called from various subsystems to register and update variables.
 */
public class FlytDashboardV2 {

    private final NetworkTable table;
    private final Map<String, BooleanPublisher> boolPublishers = new HashMap<>();
    private final Map<String, IntegerPublisher> intPublishers = new HashMap<>();
    private final Map<String, DoublePublisher> doublePublishers = new HashMap<>();
    private final Map<String, StringPublisher> stringPublishers = new HashMap<>();

    /**
     * Constructs a new dashboard manager for a specific NetworkTable.
     *
     * @param tableName The name of the NetworkTable to use (e.g., "drive", "arm", etc.).
     */
    public FlytDashboardV2(String tableName) {
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    /**
     * Publishes a boolean value to the dashboard.
     *
     * @param key The key to identify the value.
     * @param value The boolean value to send to the dashboard.
     */
    public void putBoolean(String key, boolean value) {
        table.getEntry(key).setBoolean(value);
    }

    /**
     * Gets a boolean value from the dashboard, providing a default if not found.
     *
     * @param key The key to identify the value.
     * @param defaultValue The default value to return if the key is not found.
     * @return The boolean value from the dashboard or the default value.
     */
    public boolean getBoolean(String key, boolean defaultValue) {
        return table.getEntry(key).getBoolean(defaultValue);
    }

    /**
     * Publishes a double value to the dashboard.
     *
     * @param key The key to identify the value.
     * @param value The double value to send to the dashboard.
     */
    public void putDouble(String key, double value) {
        table.getEntry(key).setDouble(value);
    }

    /**
     * Gets a double value from the dashboard, providing a default if not found.
     *
     * @param key The key to identify the value.
     * @param defaultValue The default value to return if the key is not found.
     * @return The double value from the dashboard or the default value.
     */
    public double getDouble(String key, double defaultValue) {
        return table.getEntry(key).getDouble(defaultValue);
    }

    /**
     * Publishes an integer value to the dashboard.
     *
     * @param key The key to identify the value.
     * @param value The integer value to send to the dashboard.
     */
    public void putInteger(String key, int value) {
        table.getEntry(key).setInteger(value);
    }

    /**
     * Gets an integer value from the dashboard, providing a default if not found.
     *
     * @param key The key to identify the value.
     * @param defaultValue The default value to return if the key is not found.
     * @return The integer value from the dashboard or the default value.
     */
    public int getInteger(String key, int defaultValue) {
        return (int) table.getEntry(key).getInteger(defaultValue);
    }

    /**
     * Publishes a string value to the dashboard.
     *
     * @param key The key to identify the value.
     * @param value The string value to send to the dashboard.
     */
    public void putString(String key, String value) {
        table.getEntry(key).setString(value);
    }

    /**
     * Gets a string value from the dashboard, providing a default if not found.
     *
     * @param key The key to identify the value.
     * @param defaultValue The default value to return if the key is not found.
     * @return The string value from the dashboard or the default value.
     */
    public String getString(String key, String defaultValue) {
        return table.getEntry(key).getString(defaultValue);
    }

    /**
     * A simple example of how to use the FlytDashboard class.
     * You would use this logic inside your robot's main code, not as a standalone main method.
     */
    public static void main(String[] args) {
        // --- Example Usage in a Robot Class or Subsystem ---

        // Create a FlytDashboard object for the "RobotStates" table.
        FlytDashboardV2 robotDashboard = new FlytDashboardV2("RobotStates");

        // --- Sending data to the dashboard ---
        System.out.println("Putting values to dashboard...");
        double currentSpeed = 5.67;
        boolean hasCoral = true;
        String robotStatus = "Ready to intake";

        robotDashboard.putDouble("CurrentSpeed", currentSpeed);
        robotDashboard.putBoolean("HasCoral", hasCoral);
        robotDashboard.putString("RobotStatus", robotStatus);

        System.out.println("Values sent to the 'RobotStates' table.");

        // --- Getting data from the dashboard ---
        System.out.println("Getting values from dashboard...");
        double desiredSpeed = robotDashboard.getDouble("DesiredSpeed", 0.0);
        boolean intakeOverride = robotDashboard.getBoolean("IntakeOverride", false);

        System.out.println("Received DesiredSpeed: " + desiredSpeed);
        System.out.println("Received IntakeOverride: " + intakeOverride);
    }
}
