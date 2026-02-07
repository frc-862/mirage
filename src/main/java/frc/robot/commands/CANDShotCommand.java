package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.util.leds.LEDCommand;

public class CANDShotCommand extends LEDCommand {
    
    public record CANDShot(Angle hoodAngle, Angle turretAngle, AngularVelocity shooterSpeed) {}

    public static final CANDShot HUB_SHOT = new CANDShot(Degrees.of(70), Degrees.of(0), RotationsPerSecond.of(10)); //placeholder value

    protected CANDShot shot;

    private Shooter shooter;
    private Hood hood;

    public CANDShotCommand(CANDShot shot, Shooter shooter, Hood hood) {
        this.shot = shot;
        this.shooter = shooter;
        this.hood = hood;

        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(shot.shooterSpeed());
        hood.setPosition(shot.hoodAngle());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
        hood.stop();

        succeeded(!interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooter.isOnTarget() && hood.isOnTarget();
    }
}