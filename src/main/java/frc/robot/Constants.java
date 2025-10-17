package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Constants {
  public final class ElevatorConstants {

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 60;

    public final class Control {
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double CRUISE_VELOCITY = 1.5;
        public static final double ACCELERATION = 0.7;
    }

    public static final int LEFT_MOTOR_ID = 21;
    public static final int RIGHT_MOTOR_ID = 20;
    public static final double UPPER_LIMIT = 1.1;
    public static final double LOWER_LIMIT = 0;
    public static final double POSITION_TOLERANCE = 0.05;
    public static final double POS_CONVERSION = 0.04318;
    public static final double VEL_CONVERSION = 0.04318/60;
    public static final int ROTATE_CURRENT_LIMIT = 45;
    public static final int GRAB_CURRENT_LIMIT = 45;

}
}
