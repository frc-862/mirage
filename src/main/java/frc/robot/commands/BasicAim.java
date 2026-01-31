// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BasicAim extends Command {
    Turret turret;
    Angle angle;

    /**
     * @param turret a turret from the Turret subsytem to call setAngle
     * @param angle an angle to pass into the turret setAngle method
     */
    public BasicAim(Turret turret, Angle angle) {
        this.turret = turret;
        this.angle = angle;

        addRequirements(turret);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        turret.setAngle(angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * dutycycleout command for shooter
     * @param power
     * @return the command for running the shooter
     */
    public Command aimCommand() {
        return new StartEndCommand(() -> turret.setAngle(angle), () -> turret.stop(), turret);
    }
}
