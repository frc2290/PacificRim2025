package frc.robot.FLYTLib.FLYTDashboard.OldStuff;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FLYTLib.GlobalVar;
import frc.robot.FLYTLib.FLYTMotorLib.FlytMotorController;

public class MotorDashboard extends SuperDashboard {
    
    //Network tables for the controller configuration
    NetworkTable table;
    FlytMotorController controller;
    private NetworkTableEntry kP, kI, kD, kFF, set,cType;
    
    //constructor, just needs motor controller object
    public MotorDashboard(FlytMotorController m_controller){
        controller = m_controller;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Motor" + String.valueOf(controller.getMotorID()));
        kP = table.getEntry("kP");
        kI = table.getEntry("kI");
        kD = table.getEntry("kD");
        kFF = table.getEntry("kFF");
        set = table.getEntry("SetPoint");
        cType = table.getEntry("ControlType");
        kP.setDouble(0);
        kI.setDouble(0);
        kD.setDouble(0);
        kFF.setDouble(0);
        set.setDouble(0);
        controller.pidSetup(-1, 1, 0, 0, true,0);
    }

    public void test() {
        System.out.println("Howdy");
    }


    //works on printing motor status
    private void motorState(){
        set("MotorID", controller.getMotorID());
        set("MotorPosition", controller.getPos());
        set("MotorVelocity", controller.getVel());
        set("MotorVoltage", controller.getVol());
        set("MotorCurrent", controller.getCurrent());
        set("MotorTempreture", controller.getTemp());

    }

    //tunes the motor gains
    private void motorTune(double kp, double ki, double kd, double ff){
        controller.pidTune(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0), kFF.getDouble(0));
    }


    //set things on the dashboard
    private void set(String name, double value){
        table.getEntry(name).setValue(value);
    }



    @Override
    public void periodic() {
        if(GlobalVar.debug) {
            motorState();
        }
        if(GlobalVar.debug) {
            motorTune(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0), kFF.getDouble(0.0));
            controller.set(set.getDouble(0.0));
        }
    }
    


}
