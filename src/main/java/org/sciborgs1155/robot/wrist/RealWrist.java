package org.sciborgs1155.robot.wrist;

import static org.sciborgs1155.robot.Ports.Wrist.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealWrist implements WristIO {
  // 254 used a CANifier for their encoder (among other things) and I'm not doing that
  private final TalonFX pivot;
  private final DigitalInput limitSwitch;

  public RealWrist() {
    // pivot configs
    pivot = new TalonFX(PIVOT);

    TalonFXConfiguration toApply = new TalonFXConfiguration();
    toApply.CurrentLimits.SupplyCurrentLimitEnable = true;
    toApply.CurrentLimits.SupplyCurrentLimit = 50;
    toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivot.getConfigurator().apply(toApply);

    pivot.setSafetyEnabled(true);

    // limit switch
    limitSwitch = new DigitalInput(SWITCH);
  }

  @Override
  public void setVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  // will ask about conversions
  @Override
  public double getPosition() {
    return pivot.getPosition().getValueAsDouble();
  }

  @Override
  public double getVelocity() {
    return pivot.getVelocity().getValueAsDouble();
  }

  @Override
  public boolean atLimitSwitch() {
    return limitSwitch.get();
  }

  @Override
  public void zeroEncoders() {
    pivot.setPosition(0);
  }
}
