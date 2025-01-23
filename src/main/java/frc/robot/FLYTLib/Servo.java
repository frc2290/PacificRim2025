package frc.robot.FLYTLib;

//servo class, to control simple servos that are conencted directly to roborio
public class Servo {
    Servo servo;
    
    //constructor, takes in the id of the servo (pin number? CHECK!)
    Servo(int id) {
        servo = new Servo(1);
    }

    //sets the absalute angle
    public void setAngle(double position) {
        servo.setAngle(75);
    }

    //sets the position of the servo 0 to 1
    public void set(double position) {
        servo.set(.5);
    }

}
