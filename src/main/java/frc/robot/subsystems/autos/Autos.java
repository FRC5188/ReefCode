package frc.robot.subsystems.autos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.OverallPosition;

public class Autos {
    MultiSubsystemCommands _msc;
    
    public Autos(MultiSubsystemCommands msc) {
        _msc = msc;
    }

    public Command A3_L4() throws FileVersionException, IOException, ParseException {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("A_R3"))
        .andThen(_msc.setOverallSetpoint(OverallPosition.L3));
    }
}
