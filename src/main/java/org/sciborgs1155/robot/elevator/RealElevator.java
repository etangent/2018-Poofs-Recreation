package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RealElevator implements ElevatorIO {
  private final TalonFX rightLeader = new TalonFX(LEADER);
  private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER);
  private final TalonFX leftFollower1 = new TalonFX(LEFT_FOLLOWER_1);
  private final TalonFX leftFollower2 = new TalonFX(LEFT_FOLLOWER_2);
  private final Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM, SHIFTER);
  private final DigitalInput limitSwitch = new DigitalInput(SWITCH);

  public RealElevator() {
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.SupplyCurrentLimit = 30;

    rightLeader.getConfigurator().apply(toApply);
    rightFollower.getConfigurator().apply(toApply);
    leftFollower1.getConfigurator().apply(toApply);
    leftFollower2.getConfigurator().apply(toApply);

    rightFollower.setControl(new Follower(LEADER, false));
    leftFollower1.setControl(new Follower(LEADER, true));
    leftFollower2.setControl(new Follower(LEADER, true));

    // start in high gear
    shifter.set(true);
  }

  @Override
  public void setVoltage(double voltage) {
    rightLeader.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return rightLeader.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return rightLeader.getPosition().getValueAsDouble();
  }

  @Override
  public void shiftGear(boolean high) {
    shifter.set(high);
  }

  @Override
  public boolean atLimitSwitch() {
      return limitSwitch.get();
  }

  @Override
  public void zeroEncoders() {
      rightLeader.setPosition(0);
  }
}
