// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class BasicShoot extends Command {

    private Flywheel shooter;
    private double power;

    public BasicShoot(Flywheel shooter, double power) {
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setPower(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }
}
