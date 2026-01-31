// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCollect extends Command {
    private Collector collector;

    private double power;
    private Angle position;

    /**
     * Creates a new PivotCollect Command.
     *
     * @param collector
     * @param power
     * @param position
     */
    public PivotCollect(Collector collector, double power, Angle position) {
        this.collector = collector;

        this.power = power;
        this.position = position;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setCollectorPower(power);
    }

    @Override
    public void execute() {
        collector.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollector();
        collector.setPosition(Degrees.of(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}