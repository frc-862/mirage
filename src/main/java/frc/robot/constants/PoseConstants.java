package frc.robot.constants;

public class PoseConstants {

        // auton gains for posebased (if needed)
        public static final double AUTON_DRIVE_P = 1.5d;
        public static final double AUTON_DRIVE_I = 0;
        public static final double AUTON_DRIVE_D = 0.08; // 0.035
        public static final double AUTON_DRIVE_KS = 0;//0.1;

        public static final double AUTON_ROT_P = 0.03d;
        public static final double AUTON_ROT_I = 0;
        public static final double AUTON_ROT_D = 0;
        public static final double AUTON_ROT_KS = 0;

        // tele gains
        public static final double TELE_DRIVE_P = 1.5d;
        public static final double TELE_DRIVE_I = 0;
        public static final double TELE_DRIVE_D = 0.08;
        public static final double TELE_DRIVE_TOLERANCE = 0.025;
        public static final double TELE_DRIVE_KS = 0;//0.08;

        public static final double TELE_ROT_P = 0.03;
        public static final double TELE_ROT_I = 0;
        public static final double TELE_ROT_D = 0;
        public static final double TELE_ROT_TOLERANCE = 1.5;
        public static final double TELE_ROT_KS = 0; // 0.01 NOT APPLIED

        public static final double DEPLOY_VEL = 0.2; // 0.45
        public static final double BARGE_DEPLY_VEL = 0.25;
    }


