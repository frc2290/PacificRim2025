package frc.robot.FLYTLib.FLYTDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FLYTLib.GlobalVar;
import frc.robot.FLYTLib.FLYTMotorLib.SuperController;

public class MotorDashboard extends SuperDashboard{
    
    //Network tables for the controller configuration
    NetworkTable table;
    SuperController controller;
    private NetworkTableEntry kP, kI, kD, kFF;
    
    //constructor, just needs motor controller object
    public MotorDashboard(SuperController m_controller){
        controller = m_controller;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Motor" + String.valueOf(controller.getMotorID()));
        kP = table.getEntry("kP");
        kI = table.getEntry("kI");
        kD = table.getEntry("kD");
        kFF = table.getEntry("kFF");
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
    private void motorTune(){
        controller.pidTune(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0), kFF.getDouble(0));
    }


    //set things on the dashboard
    private void set(String name, double value){
        table.getEntry(name).setValue(value);
    }



    @Override
    public void periodic() {
        if(GlobalVar.debug) motorState();
        if(GlobalVar.debug) motorTune();
        // set("p", getP())
        // if (get("p") != controller.getP()) {
        //  changeP();
        // }
    }
    


}
