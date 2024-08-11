package org.sciborgs1155.robot;

public final class Ports {
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Intake {
    public static final int LEFT_ROLLER = -1;
    public static final int RIGHT_ROLLER = -1;
    public static final int LEFT_SOLENOID = -1;
    public static final int RIGHT_SOLENOID = -1;
    public static final int LEFT_BEAM = -1;
    public static final int RIGHT_BEAM = -1;
  }

  public static final class Wrist {
    public static final int PIVOT = -1;
    public static final int REMOTE_ENCODER = -1;
    public static final int SWITCH = -1;
  }

  public static final class Elevator {
    public static final int LEADER = -1;
    public static final int RIGHT_FOLLOWER = -1;
    public static final int LEFT_FOLLOWER_1 = -1;
    public static final int LEFT_FOLLOWER_2 = -1;
    public static final int REMOTE_ENCODER = -1;
    public static final int SHIFTER = -1;
    public static final int SWITCH = -1;
  }

  public static final class Forklift {
    public static final int DEPLOYER = -1;
  }

  public static final class Hanger {
    public static final int DEPLOYER = -1;
  }

  public static final class Drive {
    public static final int LEFT_LEADER = -1;
    public static final int RIGHT_LEADER = -1;
    public static final int LEFT_FOLLOWER_1 = -1;
    public static final int RIGHT_FOLLOWER_1 = -1;
    public static final int LEFT_FOLLOWER_2 = -1;
    public static final int RIGHT_FOLLOWER_2 = -1;
    public static final int GYRO= -1;
    public static final int SHIFTER = -1;
  }
}
