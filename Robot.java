// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Robot extends TimedRobot {

  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  // right motorz
  private static final WPI_VictorSPX m_rightMotor1 = new WPI_VictorSPX(1);
  private static final WPI_VictorSPX m_rightMotor2 = new WPI_VictorSPX(2);

  // LEFT motors

  private static final WPI_VictorSPX m_leftMotor1 = new WPI_VictorSPX(3);
  private static final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(4);

  private final MotorControllerGroup m_rightMotorgroup = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
  private final MotorControllerGroup m_leftMotorgroup = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

  // drivetrain
  DifferentialDrive drivetrain = new DifferentialDrive(m_leftMotorgroup, m_rightMotorgroup);

  // joystick /arcade drive
  Joystick m_joystick = new Joystick(1);

  // xbox
  XboxController m_Controller = new XboxController(0);

  // solenoid
  DoubleSolenoid m_Gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  // The next few couple of lines r copied from wpilib on how to make and run the
  // motors
  // which should work for elevator if u map it to controller

  // Spark spark = new Spark(0); // 0 is the RIO PWM port this is connected to

  // spark.set(-0.75); // the % output of the motor, between -1 and 1

  // VictorSP victor = new VictorSP(0); // 0 is the RIO PWM port this is connected
  // to

  // victor.set(0.6); // the % output of the motor, between -1 and 1

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightDrive.setInverted(true);
    m_leftMotorgroup.setInverted(false);
    m_rightMotorgroup.setInverted(true);

    // Compressor (not sure how much this is needed but `\_()_/` )
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    // Drive forwards half speed, make sure to turn input squaring off
    // m_robotDrive.arcadeDrive(0.5, 0.0, false);
    // } else {
    // m_robotDrive.stopMotor(); // stop robot
    // }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // Arcade drive with the joystick
    drivetrain.arcadeDrive(m_joystick.getY(), m_joystick.getZ());

    // Regular drive train xbox controller
    drivetrain.tankDrive(m_Controller.getLeftY(), m_Controller.getRightY());

    // Controls 4 gripper
    if (m_Controller.getAButton()) {
      m_Gripper.set(Value.kForward);
    } else {
      m_Gripper.set(Value.kReverse);
    }
    ;

    if (m_Controller.getBButton()) {
      m_Gripper.set(Value.kReverse);
    }
    ;

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
