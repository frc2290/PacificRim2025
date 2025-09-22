package frc.utils.FLYTLib.FLYTDashboard;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;
import java.util.function.Supplier;

/** Represents a single value published to the FLYT dashboard. */
public class FlytDashboardVariable {

  private String name;
  private boolean debug;

  /** Cached supplier used when the variable publishes boolean values. */
  private Supplier<Boolean> boolSupplier;

  private Supplier<Integer> intSupplier;
  private Supplier<Double> doubleSupplier;
  private Supplier<String> stringSupplier;

  /** Keeps track of the active data type so the update() switch can publish correctly. */
  private int type = 0;

  private Topic topic;
  private GenericPublisher publisher;
  private DoubleSubscriber doubleSubscriber;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public FlytDashboardVariable(String m_name, boolean m_debug) {
    name = m_name;
    debug = m_debug;
  }

  public boolean debugOnly() {
    return debug;
  }

  // creat a subsrciber for named topic
  public void addDoubleSubscriber(String name) {
    doubleSubscriber =
        inst.getDoubleTopic(name).subscribe(0.0); // we can place defult values some time later
  }

  // get the value of the subscriber
  public double getDouble() {
    return doubleSubscriber.get();
  }

  public void addBooleanSupplier(Supplier<Boolean> m_supplier) {
    boolSupplier = m_supplier;
    topic = inst.getBooleanTopic(name);
    publisher = topic.genericPublish("boolean");
  }

  public void addIntegerSupplier(Supplier<Integer> m_supplier) {
    intSupplier = m_supplier;

    topic = inst.getIntegerTopic(name);
    publisher = topic.genericPublish("int");
    type = 1;
  }

  public void addDoubleSupplier(Supplier<Double> m_supplier) {
    doubleSupplier = m_supplier;
    topic = inst.getDoubleTopic(name);
    publisher = topic.genericPublish("double");
    type = 2;
    addDoubleSubscriber(name);
  }

  public void addStringSupplier(Supplier<String> m_supplier) {
    stringSupplier = m_supplier;
    topic = inst.getStringTopic(name);
    publisher = topic.genericPublish("string");
    type = 3;
  }

  public String getName() {
    return name;
  }

  public int getType() {
    return type;
  }

  public void update(boolean m_override) {
    if (!debugOnly() || m_override) {
      switch (type) {
        case 0:
          publisher.setBoolean(boolSupplier.get());
          break;
        case 1:
          publisher.setInteger(intSupplier.get());
          break;
        case 2:
          publisher.setDouble(doubleSupplier.get());
          break;
        case 3:
          publisher.setString(stringSupplier.get());
          break;
        default:
          publisher.setString("Issue in FlytDashboardVariable Class");
          break;
      }
    }
  }
}
