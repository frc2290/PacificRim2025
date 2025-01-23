package frc.robot.FLYTLib.FLYTDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.FLYTLib.GlobalVar;
import frc.robot.FLYTLib.FLYTMotorLib.SuperController;

public class MotorDashboard extends SuperDashboard{
    
    NetworkTable table;
    SuperController controller;
    private NetworkTableEntry kP, kI, kD, kFF;
    
    public MotorDashboard(SuperController m_controller){
        controller = m_controller;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Motor" + String.valueOf(controller.getMotorID()));
    }


    
    private void update(){
        motorState();
    }

    //works on printing motor states
    private void motorState(){
        set("MotorID", controller.getMotorID());
        set("MotorVoltageComp", controller.getPos());
        set("MotorCurrentLim", controller.getVel());
        set("MotorTemp", controller.getVol());
        set("MotorCurrent", controller.getCurrent());
        set("MotorVoltage", controller.getTemp());

    }

    private void motorTune(){
        controller.pidTune(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0), kFF.getDouble(0));

    }


    //set things on the dashboard
    private void set(String name, double value){
        table.getEntry(name).setValue(value);
    }



    @Override
    public void periodic() {
        if(GlobalVar.debug) update();
        if(GlobalVar.debug) motorTune();
    }
    


}
