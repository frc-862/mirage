// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private Shooter shooter;
    private AngularVelocity velocity;

    /**
     * Basic command for moving the shooter
     * @param shooter The Shooter subsystem
     * @param velocity Motor velocity
     */
    public Shoot(Shooter shooter, AngularVelocity velocity) {
        this.shooter = shooter;
        this.velocity = velocity;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return shooter.velocityOnTarget();
    }
}
