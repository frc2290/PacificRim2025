package frc.robot.FLYTLib;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.FLYTLib.FLYTDashboard.PDHDashboard;

public class RobotSystem extends FLYTPeriodic{



    //set naviax2mxp gyro on spi
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    //Rev power distribution panel
    private PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);//set it up
    private PDHDashboard PDHDash = new PDHDashboard(PDH); //initialize the dashboard

    
    //Network tables for the controller configuration
    NetworkTable table;
    private NetworkTableEntry kGyroOffest;


    private double gyro_yawOffset = 0.0; //set gyro offset

    public RobotSystem(){
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            table = inst.getTable("Gyro");
    
            //set up the gyro
            gyro.reset();

    }




    /**
     * Returns the current yaw angle of the gyro
     * @return
     */
    public double gyro_getYaw(){
        return (gyro.getYaw() + gyro_yawOffset + 360) % 360;

    }

    /**
     * Takes in the angle offset to compensate gyro drifft or offset
     * @param angle
     */
    public void gyro_setYaw(double angle){
        gyro_yawOffset = angle - gyro.getYaw();
    }


    @Override
    public void periodic(){

        if(GlobalVar.debug){
                    //update the gyro offset
        gyro_setYaw(kGyroOffest.getDouble(0.0));

        
        //update the network tables
        table.getEntry("GyroYaw").setDouble(gyro.getYaw());
        table.getEntry("GyroYawOffset").setDouble(gyro_yawOffset);
            
        }

        
    }



}
