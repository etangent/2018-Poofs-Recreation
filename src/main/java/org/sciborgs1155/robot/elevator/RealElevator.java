package org.sciborgs1155.robot.elevator;

import static org.sciborgs1155.robot.Ports.Elevator.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RealElevator implements ElevatorIO {
  private final TalonFX leader = new TalonFX(LEADER);
  private final TalonFX follower1 = new TalonFX(FOLLOWER_1);
  private final TalonFX follower2 = new TalonFX(FOLLOWER_2);
  private final TalonFX follower3 = new TalonFX(FOLLOWER_3);
  private final Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM, SHIFTER);

  public RealElevator() {
    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.SupplyCurrentLimit = 30;

    leader.getConfigurator().apply(toApply);
    follower1.getConfigurator().apply(toApply);
    follower2.getConfigurator().apply(toApply);
    follower3.getConfigurator().apply(toApply);

    follower1.setControl(new Follower(LEADER, false));
    follower2.setControl(new Follower(LEADER, false));
    follower3.setControl(new Follower(LEADER, false));

    // start in high gear
    shifter.set(true);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return leader.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return leader.getPosition().getValueAsDouble();
  }

  @Override
  public void shiftGear(boolean high) {
    shifter.set(high);
  }
}
