package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class PoseUtils {
    public static int getSpeakerTag() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red ? 4 : 7;
        } else {
            return 4;
        }
    }

    public static boolean inRange(double range) {
        return (1 <= range || range <= 6);
    }
}
