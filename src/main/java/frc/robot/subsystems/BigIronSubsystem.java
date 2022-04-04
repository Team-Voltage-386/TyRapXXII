package frc.robot.subsystems;

import frc.robot.Logger;
import frc.robot.Robot;
import frc.robot.Utils;
import frc.robot.Constants.ShooterData;
import frc.robot.Utils.Flags;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.util.Color;
import static frc.robot.Constants.BigIronConstants.*;

/**
 * Subsystem of the BIG IRON
 * (Ball movement stuff)
 * @author Carl C.
 */
public class BigIronSubsystem extends SubsystemBase {
    // motors
    private final CANSparkMax intakeMotor = new CANSparkMax(kIntakeID, MotorType.kBrushless);
    private final CANSparkMax drumOneMotor = new CANSparkMax(kDrumOneID, MotorType.kBrushless);
    private final CANSparkMax drumTwoMotor = new CANSparkMax(kDrumTwoID, MotorType.kBrushless);
    private final TalonSRX hoodMotor = new TalonSRX(kHoodID);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoderPin);
    private final TalonSRX beltMotor = new TalonSRX(kBeltID);

    // solenoids
    private final DoubleSolenoid intakeForward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, kChannelIntakeForwardGo, kChannelIntakeForwardVent);
    private final DoubleSolenoid intakeBackward = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, kChannelIntakeBackwardGo, kChannelIntakeBackwardVent);
    private final static Value kGo = Value.kForward;
    private final static Value kVent = Value.kReverse;

    // Sensors
    private final DigitalInput breachSensor = new DigitalInput(kBreachSensorPin);
    private final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final AnalogInput pressureSensor = new AnalogInput(3);// airTank sensor
    private final DigitalInput hoodLimit = new DigitalInput(kHoodDownLimitPin);

    double tankPressure;

    // Public values
    public boolean fireTheBigIron = false;
    public boolean drumControllerOn = false;
    public boolean climbing = false;
    public boolean drumIdle = false;
    public double drumCurrentSpeed = 0;
    public double drumSP = 2000;
    public boolean ejectBall = false;
    public boolean runBeltMan = false;
    public boolean runHoodMan = false;
    public String ball1Col = "null";
    public String ball2Col = "null";
    public double hoodManPower = 0;
    public int ballCount = 0;
    public String colorOurs;
    public String colorAnti;

    // Misc.
    private final Timer ejectTimer = new Timer();
    private final Timer ledTimer = new Timer();

    // Private process variables
    private boolean calibrated = false;
    public double hoodSet = 0.05;
    private double hoodCurrentPosition = 0;
    private boolean hoodLowLimit = false;
    public boolean ballOnTheWay = false;
    public boolean breachSensorFlag = false;
    private boolean intakeSensorFlag = false;

    /** Creates a BigIronSubsystem */
    public BigIronSubsystem() {
        dPID.reset();
        hPID.reset();
        hoodMotor.configNeutralDeadband(0);
        intakeOut = false;

        // drum motor stuff
        //drumLeadMotor.restoreFactoryDefaults();
        //drumFollowMotor.restoreFactoryDefaults();
        drumOneMotor.setInverted(false);
        drumTwoMotor.setInverted(false);
        drumTwoMotor.follow(drumOneMotor,true);

        //led stuff
        ledTimer.start();
        ledA.setLength(16);
        ledA.start();
    }

    /** Used to correct a state error;
     * if bc == 0, attempt to intake (trigger intake artifically);
     * if bc == 1, reset systems and set bc = 0;
     * if bc == 2, set bc = 1, and wind ball down (used to re-intake second ball)
     */
    public void ballFailedDebug() {
        if (ballCount == 0) {
            ballOnTheWay = true;
            woundBack = false;
            beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 1) {
            ballCount = 0;
            ballOnTheWay = false;
            woundBack = false;
            ejectBall = false;
            ef = false;
            eff = false;
            beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 2) {
            ballCount = 1;
            woundBack = false;
        }
    }

    /** blanket reset */
    public void reset() {
        dPID.reset();
        hPID.reset();
        beltTimer.stop();
        beltTimer.reset();
        ejectTimer.stop();
        ejectTimer.reset();
        ejectBall = false;
        calibrated = false;
        //drumIdle = false;
        ball1Col = "null";
        ball2Col = "null";
        ballOnTheWay = false;
        ballCount = 0;
        intakeSensorFlag = false;
        breachSensorFlag = false;
        fireTheBigIron = false;
        ef = false;
        eff = false;
        beltMotor.set(ControlMode.PercentOutput, 0);
        drumOneMotor.set(0);
        drumTwoMotor.set(0);
    }

    /** set intake motor
     * @deprecated does nothing
     * @param b intake on/off
     */
    public void runIntake(boolean b) {
        if (intakeOut && b) intakeMotor.set(kIntakePower);
        else intakeMotor.set(0);
    }

    /** Checks for proper alignment, drumspeed, and hood position
     * @return whether or not the robot is completely ready to fire
     */
    public boolean readyToFire() {
        return Math.abs(drumSP - drumCurrentSpeed) < kDrumSpeedTolerance && Math.abs(hoodSet - hoodCurrentPosition) < kHoodPositionTolerance;
    }

    public boolean intakeOut = true;
    private Timer t = new Timer();

    /**
     * Refresh the intake logic, sets intake in/out and controls compliance
     * @param b a toggle for the intake retract and deploy, meant to be used with controller.getRawButtonPressed(k); inside the {@link frc.robot.commands.M_Teleop} command
     */
    public void intakeUpdate(boolean b) {
        if (b) {
            if (!intakeOut) {
                intakeOut = true;
                t.start();
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeOut = false;
                t.stop();
                t.reset();
                intakeForward.set(kVent);
                intakeBackward.set(kGo);
            }
        } else if (t.hasElapsed(1)) {
            if (Flags.complianceOverride) {
                intakeForward.set(kGo);
                intakeBackward.set(kVent);
            } else {
                intakeForward.set(kVent);
                intakeBackward.set(kVent);
            }
        }
        if (intakeOut) intakeMotor.set(kIntakePower);
        else intakeMotor.set(0);
    }

    /**prepares ball chute for next shot*/
    public void reLoad() {
        ball1Col = ball2Col;
        if (ballCount > 1) {
            ball2Col = "null";
            ballOnTheWay = true;
        }
        ballCount = 0;
    }

    @Override
    public void periodic() {
        readSensors();
        logic();
        runHood();
        runDrum();
        runFeedBelt();
        updateWidgets();
        setLED();
    }

    boolean ef = false;
    boolean eff = false;

    /** use for general state logic */
    private void logic() {
        if (ejectBall) {
            if (!ef) {
                ejectTimer.reset();
                ejectTimer.start();
                ef = true;
                eff = false;
            }
        }
        
        Logger.setInteger(7, ballCount);
        Logger.setDouble(10, hoodCurrentPosition);
        Logger.setInteger(11, (int)drumCurrentSpeed);
        Logger.setDouble(0, DriverStation.getMatchTime());
    }

    /** read the sensors and update their flags */
    private void readSensors() {
        hoodCurrentPosition = hoodEncoder.get();
        breachSensorFlag = !breachSensor.get();
        double inSens = intakeSensor.getProximity();
        intakeSensorFlag = (inSens > 310 || inSens < 230) && intakeOut;
        // According to Rev documentation pressure = 250 (voltageOut/voltageSupply)-25
        tankPressure = 250 * (pressureSensor.getVoltage() / 5) - 25;
        hoodLowLimit = !hoodLimit.get();
        drumCurrentSpeed = -1*drumOneMotor.getEncoder().getVelocity();
    }

    /** calibrate and control hood position */
    private void runHood() {
        if (calibrated) {
            double control = hALG.get(hoodCurrentPosition, hoodSet);
            if (!hoodLowLimit) hoodMotor.set(ControlMode.PercentOutput, control);// set that hood thing
            else {
                hoodMotor.set(ControlMode.PercentOutput, MathUtil.clamp(control, 0, 1));// limit that hood thing
                hPID.reset();
                hoodEncoder.reset();
            }
        } else { // if not calibrated run hood down until limit is triggered
            if (hoodLowLimit) {
                calibrated = true;
                hPID.reset();
                hoodEncoder.reset();
            }
            else {
                hoodMotor.set(ControlMode.PercentOutput, -0.55);
            }
        }
        //hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    /** Update and run drum speed pid loops */
    private void runDrum() {
        if (fireTheBigIron || drumIdle) drumOneMotor.set(dALG.get(drumCurrentSpeed, drumSP));
        else if (ejectBall || eff) drumOneMotor.set(kDrumEjectPower);
        else {
            drumOneMotor.set(0);
            dPID.reset();
        }
    }

    private Timer beltTimer = new Timer();
    private boolean woundBack = false;
    /** The most complex piece of code in the robot, is utterly absurd. Took forever to get working and 
     * is borderline black magic. This method controls how the feed belt is run and when. It also contains 
     * the logic that increases/decreases the ball count. Changing any part of this could completely 
     * break the robot's ability to intake balls and shoot them.
     */
    private void runFeedBelt() {
        if (fireTheBigIron) { // if firing, run belt as long as the robot is ready to fire
            if (readyToFire() && Flags.hoopLocked) {
                beltMotor.set(ControlMode.PercentOutput, kBeltPower); 
            } else beltMotor.set(ControlMode.PercentOutput, 0);
        } else if (ballCount == 0) {
            if (!ballOnTheWay) { // if the ball count is zero, wait for the intake to trigger to set the ballontheway flag
                if (!breachSensorFlag && intakeSensorFlag) {
                    ballOnTheWay = true;
                    beltTimer.start();
                    if (ball1Col.equals("null")) ball1Col = getColor();
                }
                beltMotor.set(ControlMode.PercentOutput, 0);
            } else { // if a ball is on the way, run it up until the breach sensor is triggered
                beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                if (breachSensorFlag || beltTimer.hasElapsed(5)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    ballOnTheWay = false;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    if (breachSensorFlag) {
                        woundBack = false;
                        ballCount++;
                    }
                }
            }
        } else if (ballCount == 1) {
            if (!woundBack) { // if the ball hasn't been wound back, wind it back for 0.6 seconds
                if (breachSensorFlag) beltTimer.start();
                beltMotor.set(ControlMode.PercentOutput, kBeltReversePower);
                if (beltTimer.hasElapsed(0.6)) {
                    beltTimer.stop();
                    beltTimer.reset();
                    woundBack = true;
                    beltMotor.set(ControlMode.PercentOutput, 0);
                }
            } else { // if it is wound back: 
                if (intakeSensorFlag) { //increase ball count if intake is triggered
                    ballCount++;
                } else if (ejectBall) { // if it needs to eject, wind until either the breach sensor is triggered, or if 0.5 seconds has passed
                    if (breachSensorFlag) {
                        if (ejectTimer.hasElapsed(0.5) && ef) {
                            beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                            eff = true;
                            ejectBall = false;
                        }
                        else beltMotor.set(ControlMode.PercentOutput, 0);
                    } else beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                } else if (eff) { // this eff flag keeps the belt running once the 0.5 seconds has passed and is ready to eject
                    if (!breachSensorFlag) {
                        beltMotor.set(ControlMode.PercentOutput, 0);
                        reLoad();
                        ef = false;
                        eff = ef;
                    }
                } else beltMotor.set(ControlMode.PercentOutput, 0); 
            }
        } else if (ballCount == 2) {
            if (ball2Col.equals("null")) ball2Col = getColor();
            else if (ejectBall) { // if ejecting, run similar code to the eject code for one ball
                if (ejectTimer.hasElapsed(0.5) && ef) {
                    beltMotor.set(ControlMode.PercentOutput, kBeltPower);
                    ejectBall = false;
                    eff = true;
                } else beltMotor.set(ControlMode.PercentOutput, 0);
            } else if (eff) { // is more complex as it needs to feed into the 1 ball eject
                if (!breachSensorFlag) {
                    beltMotor.set(ControlMode.PercentOutput, 0);
                    ball1Col = ball2Col;
                    ball2Col = "null";
                    ballCount = 1;
                    ef = false;
                    eff = ef;
                    fireTheBigIron = false;
                    ejectBall = true;
                    woundBack = true;
                }
            } else beltMotor.set(ControlMode.PercentOutput, 0);  
            if (!breachSensorFlag) beltMotor.set(ControlMode.PercentOutput, kBeltPower); // make sure balls are wound up and in firing position
    } 
}

    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private final NetworkTableEntry mainDist = mainTab.add("dist",0).withPosition(5,0).withSize(1, 1).getEntry();
    private final NetworkTableEntry mainBC = mainTab.add("BallCount",0).withPosition(3,0).withSize(1,1).getEntry();
    /** update the widgets, by calling for the tab "Main" in each class everything ends up on the same shuffleboard tab*/
    private void updateWidgets() {
        mainDist.setDouble(Utils.Flags.targetDistance);
        mainBC.setDouble(ballCount);
    }

    /** get the color of the ball; does not work with enough accuracy; ball color isn't used anywhere
     * @return a string for the color, "Blue" or "Red"
     */
    private String getColor() {
        double b = intakeSensor.getBlue();
        double r = intakeSensor.getRed();
        if (b > r) return "Blue";
        return "Red";
    }

    /** Sets the drum speed and hood position by interpolating between points in the {@link ShooterData}. 
     * The key to TyRapXXII's shooter.
     * @param m the distance to the target in meters as reported by the limelight
     * @author Carl C.
     */
    public void setAimDistance(double m) {
        int i = ShooterData.distances.length-1;
        for (int j = 1; j < ShooterData.distances.length; j++) {
            if (m < ShooterData.distances[j]) {
                i = j;
                break;
            }
        }
        double upper = ShooterData.distances[i];
        double lower = ShooterData.distances[i-1];
        double lerpFactor = (m-lower)/Math.abs(upper-lower);
        upper = ShooterData.drumSpeeds[i];
        lower = ShooterData.drumSpeeds[i-1];
        drumSP = (int)Utils.lerpB(lower, upper, lerpFactor);
        upper = ShooterData.hoodPositions[i];
        lower = ShooterData.hoodPositions[i-1];
        hoodSet = Utils.lerpB(lower, upper, lerpFactor);
    }




    ///LED CRAP
    AddressableLED ledA = new AddressableLED(9);

    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(16);

    Color blue = new Color(0, 0, 0.9);
    Color red = new Color(0.9,0,0);
    Color yellow = new Color(0.45,0.3,0.01);
    int lastBallCount = 0;
    int aniProg = 0;
    int clI = 0;
    private Color col = null;
    boolean aniRunning = false;
    public boolean ledsOn = false;
    /** it runs the animation for the leds. In short 
     * it simply steps through the color arrays, setting the color according to the animation progress. The 
     * animation progress is advanced every 0.3 seconds until it is finished. The color normally shows the number 
     * of balls in the robot, and does an alternating pattern when the climb is active. The Color changes according 
     * to alliance.
     */
    private void setLED() {
        if (ledsOn) {
            if (!climbing) {
                if (col == null) {
                    for (int i= 0; i < 16; i++) ledBuffer.setLED(i, yellow);
                    ledTimer.reset();
                    ledTimer.start();
                }
                if (DriverStation.getAlliance().toString() == "Blue") col = blue;
                else col = red;
                if (ledTimer.advanceIfElapsed(0.05)){
                    for (int i = 0; i < 8; i++) {
                        if (ballCount == 2 && i < aniProg) ledBuffer.setLED(i+8, col);
                        else ledBuffer.setLED(i+8, yellow);
                        if (ballCount == 1 && i < aniProg) ledBuffer.setLED(i, col);
                        else if (ballCount == 2) ledBuffer.setLED(i, col);
                        else ledBuffer.setLED(i, yellow);
                    }
                    boolean newBall = ballCount != lastBallCount;
                    if (aniRunning) {
                        if (aniProg > 7) {
                            aniRunning = false;
                        } else aniProg++;
                    }
                    if (newBall) {
                        if (ballCount != 0) aniRunning = true;
                        aniProg = 0;
                        if (ballCount < 2) for (int i= 0; i < 16; i++) ledBuffer.setLED(i, yellow);
                        if (ballCount == 2) for (int i = 0; i < 8; i++) ledBuffer.setLED(i+8, yellow);
                    }
                    lastBallCount = ballCount;
                }
            } else {
                if (ledTimer.advanceIfElapsed(0.3)) {
                    for (int i = 0; i < 8; i++) {
                        if (clI %2 != 0) {
                            ledBuffer.setLED(i*2, col);
                            ledBuffer.setLED((i*2) + 1, yellow);
                        } else {
                            ledBuffer.setLED(i*2, yellow);
                            ledBuffer.setLED((i*2) + 1, col);
                        }
                    }
                    clI++;
                }
            }
            ledA.setData(ledBuffer);
        } else {
            col = null;
            for (int i = 0; i < 16; i++) ledBuffer.setLED(i,new Color(0,0,0));
            ledA.setData(ledBuffer);
            ledTimer.stop();
        }
    }
}