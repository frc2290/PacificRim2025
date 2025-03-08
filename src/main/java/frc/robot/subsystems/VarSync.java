package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VarSync{
    
 
    private double var = 0.0;
    private String name;

    public VarSync(String m_name, double m_var){
        var = m_var;
        name = m_name;
        Preferences.initDouble(name, var);

    }

    public double getVar(){
        return var;
    }

    public boolean check(double m_value){
        if(m_value == var){
            return true;
        }else{
            return false;
        }

    }
    
    public void loadPreferences(){
        var = Preferences.getDouble(name, var);
    }
}
