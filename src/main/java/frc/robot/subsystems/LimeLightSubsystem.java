package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;
import static frc.robot.Constants.LimeLightConstants.*;

/**2022 LimeLight Code
 * @author Carl C.
 * @author Max V.
*/
public class LimeLightSubsystem extends SubsystemBase {
  /** The networkTable instance for the camera */
  public NetworkTable _nt;
  /**Whether or not the LL should wait before declaring target loss to see if it will come back*/
  public Boolean targetLostWait = true;
  public Boolean dMode = false;
  public Boolean targetFound = false;
  /** target right/left angle */
  public float tx = 0;
  /** target up/down angle */
  public float ty = 0;
  private final  Timer timer = new Timer();

  /**Create A LimeLight Object
   * @param hostName The hostname of the LimeLight, the code finds the network table with this
   * @param targetHeight the height of the intended target from the ground, used to calculate distance
   * @param mountAngle the up/down angle of the camera relative to the horizon, used to calculate distance
   * @param mountHeight the height of the LimeLight off the ground
   * @param pl the default pipeline to be used
  */
  public LimeLightSubsystem(String hostName, int pl) {
      _nt = NetworkTableInstance.getDefault().getTable(hostName);
      setPipeLine(pl);
      driverMode(false);
      timer.start();
  }

  
  /**During periodic this subsystem updates the basic public LL variables*/
  @Override
  public void periodic() {
    if(_nt.getEntry("tv").getDouble(-1) == 0) { // that '-1' saved our season, a random choice to change the default to anything other than zero
      if(targetLostWait) targetFound = !timer.hasElapsed(LimeLightConstants.targetLostWaitTime); // if it should wait for target re-acquire, then wait the tlwt, else declare it lost
      else targetFound = false;
    } else {
      targetFound = true; // if target is found, update values and keep timer reset, but running
      timer.reset();
      timer.start();
      tx = (float)_nt.getEntry("tx").getDouble(0);
      ty = (float)_nt.getEntry("ty").getDouble(0);
    }
    updateWidgets();
  }

  /**Empty*/
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Set drivermode (Exposure turned up to make image visible to mere humans)
   * @param b whether or not drivermode should be turned on or off
   */
  public void driverMode(Boolean b) {
    dMode = b;
    if (b) _nt.getEntry("camMode").setNumber(1);
    else _nt.getEntry("camMode").setNumber(0);
  }

  /**
   * Set the LimeLight Pipeline
   * @param p Pipeline Index
   */
  public void setPipeLine(Integer p) {
    _nt.getEntry("pipeline").setNumber(p);
  }

  /**Returns the meters to the target given the target's height from the ground*/
  public double metersToTarget() {
    return distALG.get(ty);
  }

  /** turn  */
  public void lights(boolean b) {
    if (b) _nt.getEntry("ledMode").setNumber(3);
    else _nt.getEntry("ledMode").setNumber(1);
  }

  // not required 
  private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private final NetworkTableEntry mainX = mainTab.add("LL-X",0).withPosition(6,0).withSize(1, 1).getEntry();
  private final NetworkTableEntry mainFound = mainTab.add("TargetFound",false).withPosition(4, 0).withSize(1, 1).getEntry();
  /** update widgets, not required, if reusing this code change it to your liking */
  private void updateWidgets() {
    mainFound.setBoolean(targetFound);
    mainX.setDouble(tx);
  }
}