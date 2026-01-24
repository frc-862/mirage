// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class BasicCollect extends Command {
    private Collector collector;
    private double power;

    /**
     * Collects fuel from the field
     *
     * @param collector The collector subsystem
     * @param power     Motor power
     */

    public BasicCollect(Collector collector, double power) {
        this.collector = collector;
        this.power = power;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }
}
