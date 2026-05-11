package frc.util.hardware;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class XboxControllerFilter extends XboxController {

    private double joystickDeadband;
    private double triggerDeadband;
    private double multiplicity;
    private DoubleSupplier multiplier;
    private double rateLimit;

    private Vector<N2> leftStickVec;
    private Vector<N2> rightStickVec;

    private SlewRateLimiter leftXSlewRateLimiter;
    private SlewRateLimiter leftYSlewRateLimiter;
    private SlewRateLimiter rightXSlewRateLimiter;
    private SlewRateLimiter rightYSlewRateLimiter;
    

    /**
     * A wrapper for XboxController that allows multiplicities, deadbands, and rate limits to be applied cleanly.
     * @param port The port number of the Xbox controller.
     */
    public XboxControllerFilter(int port) {
        super(port);

        CommandScheduler.getInstance().schedule(new RunCommand(() -> this.periodic()));

        joystickDeadband = 0;
        triggerDeadband = 0;
        multiplicity = 1;
        multiplier = () -> 1;
        rateLimit = 2;

        leftStickVec = VecBuilder.fill(0, 0);
        rightStickVec = VecBuilder.fill(0, 0);

        leftXSlewRateLimiter = new SlewRateLimiter(rateLimit);
        leftYSlewRateLimiter = new SlewRateLimiter(rateLimit);
        rightXSlewRateLimiter = new SlewRateLimiter(rateLimit);
        rightYSlewRateLimiter = new SlewRateLimiter(rateLimit);
    }

    /**
     * Calculates the filtered joystick values.
     */
    private void periodic() {

        // build vectors from joystick values
        leftStickVec.set(0, 0, super.getLeftX());
        leftStickVec.set(1, 0, super.getLeftY());
        rightStickVec.set(0, 0, super.getRightX());
        rightStickVec.set(1, 0, super.getRightY());

        // apply deadbands
        leftStickVec = MathUtil.applyDeadband(leftStickVec, joystickDeadband);
        rightStickVec = MathUtil.applyDeadband(rightStickVec, joystickDeadband);

        // apply multiplicity
        leftStickVec = leftStickVec.times(Math.pow(leftStickVec.norm(), multiplicity - 1));
        rightStickVec = rightStickVec.times(Math.pow(rightStickVec.norm(), multiplicity - 1));

        // apply multiplier
        leftStickVec = leftStickVec.times(multiplier.getAsDouble());
        rightStickVec = rightStickVec.times(multiplier.getAsDouble());
        
        // apply rate limiting
        if (rateLimit > 0){
            leftStickVec.set(0, 0, leftXSlewRateLimiter.calculate(leftStickVec.get(0)));
            leftStickVec.set(1, 0, leftYSlewRateLimiter.calculate(leftStickVec.get(1)));
            rightStickVec.set(0, 0, rightXSlewRateLimiter.calculate(rightStickVec.get(0)));
            rightStickVec.set(1, 0, rightYSlewRateLimiter.calculate(rightStickVec.get(1)));
        }
    }

    /**
     * @param deadband The deadband value for the joysticks.
     * @return This XboxControllerFilter instance for method chaining.
     */
    public XboxControllerFilter withJoystickDeadband(double deadband){
        this.joystickDeadband = deadband;
        return this;
    }

    /**
     * @param deadband The deadband value for the triggers.
     * @return This XboxControllerFilter instance for method chaining.
     */
    public XboxControllerFilter withTriggerDeadband(double deadband){
        this.triggerDeadband = deadband;
        return this;
    }

    /**
     * @param multiplicity The multiplicity value.
     * <br> typically 1, 2, or 3. Higher multiplicity values increases control at low speeds.
     * @return This XboxControllerFilter instance for method chaining.
     */
    public XboxControllerFilter withMultiplicity(double multiplicity){
        this.multiplicity = multiplicity;
        return this;
    }

    /**
     * @param multiplier The multiplier value.
     * @return This XboxControllerFilter instance for method chaining.
     */
    public XboxControllerFilter withMultiplier(double multiplier){
        this.multiplier = () -> multiplier;
        return this;
    }

    /**
     * @param multiplier The multiplier value.
     * @return This XboxControllerFilter instance for method chaining.
     */
    public XboxControllerFilter withMultiplier(DoubleSupplier multiplier){
        this.multiplier = multiplier;
        return this;
    }

    /**
     * @param rateLimit The rate-of-change limit, in units per second. 
     * <br> applied only to joystick axes. A value of 0 or less disables rate limiting.
     * @return This XboxControllerFilter instance for method chaining.
     */
    public XboxControllerFilter withRateLimit(double rateLimit){
        this.rateLimit = rateLimit;
        return this;
    }

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftX() {
        return leftStickVec.get(0);
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return leftStickVec.get(1);
    }   

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return rightStickVec.get(0);
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return rightStickVec.get(1);
    }

    /**
     * Get the left trigger axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1]. Has deadband applied.
     *
     * @return The axis value.
     */
    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(super.getLeftTriggerAxis(), triggerDeadband);
    }

    /**
     * Get the right trigger axis value of the controller. Note that this axis is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1]. Has deadband applied.
     *
     * @return The axis value.
     */
    @Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(super.getRightTriggerAxis(), triggerDeadband);
    }

    /**
     * @return true if left trigger is pressed past deadband
     */
    public boolean getLeftTrigger() {
        return getLeftTriggerAxis() > 0;
    }

    /**
     * @return true if right trigger is pressed past deadband
     */
    public boolean getRightTrigger() {
        return getRightTriggerAxis() > 0;
    }

    /**
     * @return true if DPad up is pressed
     */
    public boolean getDPadUp() {
        return getPOV() == 0;
    }

    /**
     * @return true if DPad right is pressed
     */
    public boolean getDPadRight() {
        return getPOV() == 90;
    }

    /**
     * @return true if DPad down is pressed
     */
    public boolean getDPadDown() {
        return getPOV() == 180;
    }

    
    public boolean getDPadLeft() {
        return getPOV() == 270;
    }
}