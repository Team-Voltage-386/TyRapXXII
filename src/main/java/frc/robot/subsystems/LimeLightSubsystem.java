package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;

/**Carl's attempt at making a lime-light subsystem*/
public class LimeLightSubsystem extends SubsystemBase {
  public NetworkTable _nt;
  /**Whether or not the LL should wait before declaring target loss to see if it will come back*/
  public Boolean targetLostWait = true;
  public Boolean dMode = false;
  public Boolean targetFound = false;
  public float tx = 0;
  public float ty = 0;
  private final double _TH;
  private final double _MA;
  private final double _MH;
  private final  Timer timer = new Timer();
  private final ShuffleboardTab tab = Shuffleboard.getTab("LL");
  private final NetworkTableEntry distWidget = tab.add("dist",0).withPosition(0, 0).withSize(1, 1).getEntry();

  /**Carl's attempt at making a lime-light subsystem*/
  public LimeLightSubsystem(String hostName, double targetHeight, double mountAngle, double mountHeight, int pl) {
      _nt = NetworkTableInstance.getDefault().getTable(hostName);
      _TH = targetHeight;
      _MA = mountAngle;
      _MH = mountHeight;
      setPipeLine(pl);
      driverMode(false);
      timer.start();
  }

  
  /**During periodic this subsystem updates the basic public LL variables*/
  @Override
  public void periodic() {
    if(_nt.getEntry("tv").getDouble(-1) == 0) {
      if(targetLostWait) targetFound = !timer.hasElapsed(LimeLightConstants.targetLostWaitTime); // if it should wait for target re-acquire, then wait, else declare it lost
      else targetFound = false;
    }
    else {
        targetFound = true;
        timer.reset();
        timer.start();
        tx = (float)_nt.getEntry("tx").getDouble(0);
        ty = (float)_nt.getEntry("ty").getDouble(0);
        distWidget.setDouble(metersToTarget());
    }


  }

  /**Empty*/
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Enable/Disable drivermode (Exposure turned up to make image visible to mere humans)
   * @param b whether or not drivermode should be turned on or off
   */
  public void driverMode(Boolean b) {
      dMode = b;
      if (b) _nt.getEntry("camMode").setNumber(1);
      else _nt.getEntry("camMode").setNumber(0);
  }

  public void setPipeLine(Integer p) {
    _nt.getEntry("pipeline").setNumber(p);
  }

  /**Returns the meters to the target given the target's height from the ground*/
  public double metersToTarget() {
    return (_TH-_MH)/Math.tan(Math.PI*((_MA+ty)/180));
  }

  public void lights(boolean b) {
      if (b) _nt.getEntry("ledMode").setNumber(3);
      else _nt.getEntry("ledMode").setNumber(1);
  }
}