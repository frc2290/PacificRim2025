
package frc.robot;

public final class Constants {

  public static final class Motors{
    public static final class RollerM{

      public static final int m_id = 7;
      public static final boolean m_brushless = true;
      public static boolean m_break_mode = true;

      public static double m_voltage_compensation = 10;
      public static int m_motor_current = 60;
    }
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
