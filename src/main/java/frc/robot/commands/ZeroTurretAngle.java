// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class ZeroTurretAngle extends Command {

    private final Turret turret;

    public ZeroTurretAngle(Turret turret) {
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setPower(0.5);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        turret.setAngle();
    }

    @Override
    public boolean isFinished() {
        return turret.getZeroLimitSwitch();
    }
}
