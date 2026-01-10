package frc.robot.constants;

public class LEDConstants {
    public static int LED_PWM_PORT = 0;
    
    public static int LED_COUNT = 200;

    public enum LED_STATES {
        TEST,
        ERROR,
        AUTO,
        TELEOP;
        
        public int ID() {
            return this.ordinal();
        }
    }    
}
