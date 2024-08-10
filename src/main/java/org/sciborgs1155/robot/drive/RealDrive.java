package org.sciborgs1155.robot.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.HIGH_GEARING;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.LOW_GEARING;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.WHEEL_RADIUS;

import java.util.Arrays;

public class RealDrive implements DriveIO {
    private final TalonFX leftLeader, rightLeader, leftFollower1, rightFollower1, leftFollower2, rightFollower2;
    private final Solenoid shifter;
    private double currentGearing;

    public RealDrive() {
        leftLeader = new TalonFX(LEFT_LEADER);
        rightLeader = new TalonFX(RIGHT_LEADER);
        leftFollower1 = new TalonFX(LEFT_FOLLOWER_1);
        rightFollower1 = new TalonFX(RIGHT_FOLLOWER_1);
        leftFollower2 = new TalonFX(LEFT_FOLLOWER_2);
        rightFollower2 = new TalonFX(RIGHT_FOLLOWER_2);

        TalonFX[] motors = {leftLeader, rightLeader, leftFollower1, rightFollower1, leftFollower2, rightFollower2};

        TalonFXConfiguration toApply = new TalonFXConfiguration();
        toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
        toApply.CurrentLimits.SupplyCurrentLimit = 50;

        Arrays.stream(motors).forEach((motor) -> motor.getConfigurator().apply(toApply));

        rightFollower1.setControl(new Follower(RIGHT_LEADER, false));
        rightFollower2.setControl(new Follower(RIGHT_LEADER, false));
        leftFollower1.setControl(new Follower(LEFT_LEADER, false));
        leftFollower2.setControl(new Follower(LEFT_LEADER, false));

        
        shifter = new Solenoid(PneumaticsModuleType.CTREPCM, SHIFTER);
        shifter.set(true);
        currentGearing = HIGH_GEARING;
    }

    @Override
    public void setVoltages(double leftVoltage, double rightVoltage) {
        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);
    }

    @Override
    public double getRightVelocity() {
        return rightLeader.getVelocity().getValueAsDouble() * Rotations.of(currentGearing).in(Radians) * WHEEL_RADIUS.in(Meters);
    }

    @Override
    public double getLeftVelocity() {
        return leftLeader.getVelocity().getValueAsDouble() * Rotations.of(currentGearing).in(Radians) * WHEEL_RADIUS.in(Meters);
    }

    @Override
    public void shiftGears(boolean high) {
        shifter.set(high);
        currentGearing = high == true ? HIGH_GEARING : LOW_GEARING;
    }
}
