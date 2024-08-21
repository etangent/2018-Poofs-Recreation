package org.sciborgs1155.robot.commands;

import static org.sciborgs1155.robot.Constants.*;

import org.sciborgs1155.robot.Robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.elevator.Elevator;
import org.sciborgs1155.robot.intake.Intake;
import org.sciborgs1155.robot.wrist.Wrist;

public final class Autos {
  public static SendableChooser<Command> configureAutos(
      Drive drive, Elevator elevator, Wrist wrist, Intake intake) {
    AutoBuilder.configureRamsete(
        drive::pose,
        drive::resetPose,
        drive::chassisSpeeds,
        drive::setChassisSpeeds,
        new ReplanningConfig(),
        () -> alliance() == Alliance.Red,
        drive);

    NamedCommands.registerCommand("prepare-high-shot", 
    elevator.fullExtend().until(elevator::atMaxHeight).alongWith(wrist.shootAngle()));
    NamedCommands.registerCommand("shoot", Robot.isReal() ? intake.shoot() : new WaitCommand(.5));
    NamedCommands.registerCommand("stow-elevator", elevator.stow());
    NamedCommands.registerCommand("deploy-intake", wrist.unStow());
    NamedCommands.registerCommand("intake", Robot.isReal() ? intake.intakeAndKeep() : new WaitCommand(.5));

    FollowPathCommand.warmupCommand().schedule();

    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser("3 scale");
    return chooser;
  }
}
