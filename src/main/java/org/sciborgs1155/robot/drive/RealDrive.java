package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.robot.Ports.Drive.*;
import static org.sciborgs1155.robot.drive.DriveConstants.Drivetrain.WHEEL_RADIUS;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import java.util.Arrays;

public class RealDrive implements DriveIO {
  private final TalonFX leftLeader,
      rightLeader,
      leftFollower1,
      rightFollower1,
      leftFollower2,
      rightFollower2;
  private final Pigeon2 gyro = new Pigeon2(GYRO);
  private final Solenoid shifter;

  public RealDrive() {
    leftLeader = new TalonFX(LEFT_LEADER);
    rightLeader = new TalonFX(RIGHT_LEADER);
    leftFollower1 = new TalonFX(LEFT_FOLLOWER_1);
    rightFollower1 = new TalonFX(RIGHT_FOLLOWER_1);
    leftFollower2 = new TalonFX(LEFT_FOLLOWER_2);
    rightFollower2 = new TalonFX(RIGHT_FOLLOWER_2);

    TalonFX[] motors = {
      leftLeader, rightLeader, leftFollower1, rightFollower1, leftFollower2, rightFollower2
    };

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
  }

  @Override
  public void setVoltages(double leftVoltage, double rightVoltage) {
    leftLeader.setVoltage(leftVoltage);
    rightLeader.setVoltage(rightVoltage);
  }

  /*
   * I cannot find how 254 accounts for gearing
   * (see getLeftEnoderDistance())
   * so neither will I cause it's too hard
   * cry about it
   */
  @Override
  public double getLeftPosition() {
    return leftLeader.getPosition().getValueAsDouble() * WHEEL_RADIUS.in(Meters) * 2 * Math.PI;
  }

  @Override
  public double getRightPosition() {
    return rightLeader.getPosition().getValueAsDouble() * WHEEL_RADIUS.in(Meters) * 2 * Math.PI;
  }

  @Override
  public double getRightVelocity() {
    return rightLeader.getVelocity().getValueAsDouble() * WHEEL_RADIUS.in(Meters) * 2 * Math.PI;
  }

  @Override
  public double getLeftVelocity() {
    return leftLeader.getVelocity().getValueAsDouble() * WHEEL_RADIUS.in(Meters) * 2 * Math.PI;
  }

  @Override
  public Rotation2d getHeading() {
    return gyro.getRotation2d();
  }

  @Override
  public void shiftGears(boolean high) {
    shifter.set(high);
  }

  @Override
  public void close() throws Exception {
    rightLeader.close();
    rightFollower1.close();
    rightFollower2.close();
    leftLeader.close();
    leftFollower1.close();
    leftFollower2.close();
    gyro.close();
    shifter.close();
  }
}
