package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.DRIVE_MASS;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.HIGH_GEARING;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.LOW_GEARING;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.MOI;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.STD_DEVS;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.TRACK_WIDTH;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.WHEEL_RADIUS;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class SimDrive implements DriveIO {
    private final DifferentialDrivetrainSim drive;

    public SimDrive() {
        drive = new DifferentialDrivetrainSim(DCMotor.getMiniCIM(3), HIGH_GEARING, MOI, DRIVE_MASS.in(Kilograms), WHEEL_RADIUS.in(Meters), TRACK_WIDTH.in(Meters), STD_DEVS);
    }

    @Override
    public void setVoltages(double leftVoltage, double rightVoltage) {
        drive.setInputs(leftVoltage, rightVoltage);
    }

    @Override
    public double getLeftVelocity() {
        return drive.getLeftVelocityMetersPerSecond();
    }

    @Override
    public double getRightVelocity() {
        return drive.getRightVelocityMetersPerSecond();
    }

    @Override
    public void shiftGears(boolean high) {
        if (high)  {
            drive.setCurrentGearing(HIGH_GEARING);
        } else {
            drive.setCurrentGearing(LOW_GEARING);
        }
    }
}
