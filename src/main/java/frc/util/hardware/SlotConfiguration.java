package frc.util.hardware;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
 
// This class is used to set the PIDF gains for a TalonFX Config for a specified slot number. it is primarily for backend use.
public class SlotConfiguration {
    private int slot;
    private TalonFXConfiguration config;
    
    /**
     * creates a new SlotConfiguration object.
     * @param slot specified PID slot to use: 0, 1, or 2
     * @param config TalonFXConfiguration object to apply gains to
     * @implNote any changes made to the SlotConfiguration object will not be applied to the TalonFXConfiguration object until getConfig() is called
     * @implNote any changes made to the input TalonFXConfiguration object between initialization and getConfig() will not be applied to the final TalonFXConfiguration
     */
    public SlotConfiguration(int slot, TalonFXConfiguration config) {
        this.slot = slot;
        this.config = config;

        switch (slot) {
            case 0:
                kP = config.Slot0.kP;
                kI = config.Slot0.kI;
                kD = config.Slot0.kD;
                kS = config.Slot0.kS;
                kV = config.Slot0.kV;
                kA = config.Slot0.kA;
                break;
            case 1:
                kP = config.Slot1.kP;
                kI = config.Slot1.kI;
                kD = config.Slot1.kD;
                kS = config.Slot1.kS;
                kV = config.Slot1.kV;
                kA = config.Slot1.kA;
                break;
            case 2:
                kP = config.Slot2.kP;
                kI = config.Slot2.kI;
                kD = config.Slot2.kD;
                kS = config.Slot2.kS;
                kV = config.Slot2.kV;
                kA = config.Slot2.kA;
                break;
        }
    }


    /**
     * gets the TalonFXConfiguration object input, plus any gains specified
     * @return TalonFXConfiguration object with the specified gains
     */
    public TalonFXConfiguration getConfig() {
        switch (slot) {
            case 0:
                config.Slot0.kP = kP;
                config.Slot0.kI = kI;
                config.Slot0.kD = kD;
                config.Slot0.kS = kS;
                config.Slot0.kV = kV;
                config.Slot0.kA = kA;
                break;
            case 1:
                config.Slot1.kP = kP;
                config.Slot1.kI = kI;
                config.Slot1.kD = kD;
                config.Slot1.kS = kS;
                config.Slot1.kV = kV;
                config.Slot1.kA = kA;
                break;
            case 2:
                config.Slot2.kP = kP;
                config.Slot2.kI = kI;
                config.Slot2.kD = kD;
                config.Slot2.kS = kS;
                config.Slot2.kV = kV;
                config.Slot2.kA = kA;
                break;
        }
        return config;
    }
    
    
    /**
     * Proportional Gain
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by error in the input, the units should be
     * defined as units of output per unit of input error. For example,
     * when controlling velocity using a duty cycle closed loop, the units
     * for the proportional gain will be duty cycle per rps, or 1/rps.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 3.4e+38
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kP = 0;
    /**
     * Integral Gain
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by error in the input integrated over time
     * (in units of seconds), the units should be defined as units of
     * output per unit of integrated input error. For example, when
     * controlling velocity using a duty cycle closed loop, integrating
     * velocity over time results in rps * s = rotations. Therefore, the
     * units for the integral gain will be duty cycle per rotation of
     * accumulated error, or 1/rot.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 3.4e+38
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kI = 0;
    /**
     * Derivative Gain
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by the derivative of error in the input
     * with respect to time (in units of seconds), the units should be
     * defined as units of output per unit of the differentiated input
     * error. For example, when controlling velocity using a duty cycle
     * closed loop, the derivative of velocity with respect to time is
     * rps/s, which is acceleration. Therefore, the units for the
     * derivative gain will be duty cycle per unit of acceleration error,
     * or 1/(rps/s).
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 3.4e+38
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kD = 0;
    /**
     * Static Feedforward Gain
     * <p>
     * This is added to the closed loop output. The sign is determined by
     * target velocity. The unit for this constant is dependent on the
     * control mode, typically fractional duty cycle, voltage, or torque
     * current.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -512
     *   <li> <b>Maximum Value:</b> 511
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kS = 0;
    /**
     * Velocity Feedforward Gain
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by the requested velocity, the units should
     * be defined as units of output per unit of requested input velocity.
     * For example, when controlling velocity using a duty cycle closed
     * loop, the units for the velocity feedfoward gain will be duty cycle
     * per requested rps, or 1/rps.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 3.4e+38
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kV = 0;
    /**
     * Acceleration Feedforward Gain
     * <p>
     * The units for this gain is dependent on the control mode. Since
     * this gain is multiplied by the requested acceleration, the units
     * should be defined as units of output per unit of requested input
     * acceleration. For example, when controlling velocity using a duty
     * cycle closed loop, the units for the acceleration feedfoward gain
     * will be duty cycle per requested rps/s, or 1/(rps/s).
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 3.4e+38
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kA = 0;
    /**
     * Gravity Feedforward Gain
     * <p>
     * This is added to the closed loop output. The sign is determined by
     * the type of gravity feedforward. The unit for this constant is
     * dependent on the control mode, typically fractional duty cycle,
     * voltage, or torque current.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> -512
     *   <li> <b>Maximum Value:</b> 511
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> 
     *   </ul>
     */
    public double kG = 0;
    /**
     * Gravity Feedforward Type
     * <p>
     * This determines the type of the gravity feedforward. Choose
     * Elevator_Static for systems where the gravity feedforward is
     * constant, such as an elevator. The gravity feedforward output will
     * always be positive. Choose Arm_Cosine for systems where the gravity
     * feedforward is dependent on the angular position of the mechanism,
     * such as an arm. The gravity feedforward output will vary depending
     * on the mechanism angular position. Note that the sensor offset and
     * ratios must be configured so that the sensor position is 0 when the
     * mechanism is horizonal, and one rotation of the mechanism
     * corresponds to one rotation of the sensor position.
     * 
     */
    public GravityTypeValue GravityType = GravityTypeValue.Elevator_Static;
}