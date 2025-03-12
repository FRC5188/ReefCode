package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class HardwareConstants {

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public class CAN {
        // Add CAN IDs here
        public static final int PDH_ID = 1;

        public static final int ARM_MTR_ID = 14;
        public static final int INTAKE_MTR_ID = 15;
        public static final int CLIMBER_MTR_ID = 30;

        public static final int PRIMARY_ELEVATOR_ID = 16;
        public static final int SECONDARY_ELEVATOR_ID = 17;

        public static final int CANDLE_ID = 50;

        public static final int CORAL_LASERCAN_ID = 20;
    }

    public class DIO {
        // Add digitial I/O ports used here
        public static final int LIGHT_SENSOR_CHANNEL = 0;
    }

    // Use LoggedTunableNumbers
    public static final boolean tuningMode = true;
}
