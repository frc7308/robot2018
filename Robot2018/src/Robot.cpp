/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <Joystick.h>
#include <RobotDrive.h>
#include <Servo.h>
#include <WPILib.h>

class Robot : public frc::IterativeRobot {
public:
	RobotDrive *drive;
	Joystick *stick;
	SmartDashboard *dashboard;

	Servo *armServo0;
	Servo *armServo1;
	Servo *armServo2;
	Servo *armServo3;
	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		drive = new RobotDrive(1, 2, 3, 4);

		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			drive->Drive(1.0, 0.0);
		}
	}

	void TeleopInit() {
		stick = new Joystick(5);
		drive = new RobotDrive(1, 2, 3, 4);
	}

	void TeleopPeriodic() {
		while (IsOperatorControl() && IsEnabled()) {
			drive->Drive(0.25, 0.0);
			//drive->ArcadeDrive(stick);
			//dashboard->PutNumber("test", 5.0);
			Wait(0.01);
		}
	}

	void TestPeriodic() {
		drive->Drive(0.25, 0.0);
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	void lowArmPosition() {
		armServo0->SetAngle(0.0);
		armServo1->SetAngle(180.0);
		armServo2->SetAngle(0.0);
		armServo3->SetAngle(330);
	}

	void highArmPosition() {
		armServo0->SetAngle(90.0);
		armServo1->SetAngle(90.0);
		armServo2->SetAngle(90.0);
		armServo3->SetAngle(0);
	}
};

START_ROBOT_CLASS(Robot)
