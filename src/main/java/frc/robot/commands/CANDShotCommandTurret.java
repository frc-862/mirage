package frc.robot.commands;

import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class CANDShotCommandTurret extends CANDShotCommand {

    private Turret turret;

    public CANDShotCommandTurret(CANDShot shot, Shooter shooter, Hood hood, Turret turret) {
        super(shot, shooter, hood);

        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setAngle(shot.turretAngle());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        turret.stop();

        succeeded(!interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return super.isFinished() && turret.isOnTarget();
    }
}