package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants.CannedShot;

public class CannedShots extends Command {
    private final CannedShot shot;

    public CannedShots(CannedShot shot){
        this.shot = shot;
    }

    @Override
    public void initialize() {

        switch (shot) {
            case HUB:
                break;
            case SQUARE_THING:
                break;
            case CORNER_BOTTOM:
                break;
            case CORNER_TOP:
                break;
            case TUNNEL_ENTRANCE_BOTTOM:
                break;
            case TUNNEL_ENTRANCE_TOP:
                break;
        }
    }

    @Override
    public void execute() {
    }
}
