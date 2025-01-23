package frc.robot.FLYTLib.FLYTDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.FLYTLib.GlobalVar;


public class PDHDashboard extends SuperDashboard{
    
    //Network tables for the controller configuration
    NetworkTable table;
    PowerDistribution PDH;
    
    //constructor, just needs motor controller object
    public PDHDashboard(PowerDistribution PDH){
        this.PDH = PDH;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("PowerDistributionHub");
    }


    //get PDH status
    private void PDHstate(){
        PDH.getVoltage();
        PDH.getTotalCurrent();
        PDH.getTemperature();
        PDH.getCurrent(0);
        PDH.getCurrent(1);
        PDH.getCurrent(2);
        PDH.getCurrent(3);
        PDH.getCurrent(4);
        PDH.getCurrent(5);
        PDH.getCurrent(6);
        PDH.getCurrent(7);
        PDH.getCurrent(8);
        PDH.getCurrent(9);
        PDH.getCurrent(10);
        PDH.getCurrent(11);
        PDH.getCurrent(12);
        PDH.getCurrent(13);
        PDH.getCurrent(14);
        PDH.getCurrent(15);
        PDH.getCurrent(16);
        PDH.getCurrent(17);
        PDH.getCurrent(18);
        PDH.getCurrent(19);
        PDH.getCurrent(20);
        PDH.getCurrent(21);
        PDH.getCurrent(22);
        PDH.getCurrent(23);
    }




    @Override
    public void periodic() {
        if(GlobalVar.debug) PDHstate();
    }
    


}
