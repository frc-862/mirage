// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class Collect extends Command {
    private Collector collector;
    private double power;
    private Angle position = Radians.of(0);

    /**
     * Basic command for moving the rollers of the collector
     * @param collector The collector subsystem
     * @param power Motor power
     * @param position position
     */
    public Collect(Collector collector, double power, Angle position) {
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
    collector.setPivotAngle(position);
  }

    @Override
    public void end(boolean interrupted) {
        collector.stopCollector();
    }
}
