package frc.robot.FLYTLib.FLYTDashboard;

import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;

public class FlytDashboardVariable {

    private String name;
    private Supplier<Boolean> boolSupplier;
    private Supplier<Integer> intSupplier;
    private Supplier<Double> doubleSupplier;
    private Supplier<String> stringSupplier;
    private int type = 0;
    private Topic topic;
    private GenericPublisher publisher;
    private DoubleSubscriber doubleSubscriber;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public FlytDashboardVariable(String m_name) {
        name = m_name;
    }

    //creat a subsrciber for named topic
    public void addDoubleSubscriber(String name){
        doubleSubscriber = inst.getDoubleTopic(name).subscribe(0.0);//we can place defult values some time later
    }

    //get the value of the subscriber
    public double getDouble(){
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

    public void update() {
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
 