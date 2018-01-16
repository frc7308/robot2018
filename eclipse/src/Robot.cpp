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
#include <Timer.h>
#include <WPILib.h>

class Robot : public frc::IterativeRobot {
public:
	Joystick *stick;
	RobotDrive *drive;
	Timer *timer;
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
		drive = new RobotDrive(0, 1, 2, 3);
		drive->SetSafetyEnabled(false);

		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			MoveForTime(3.0, 1, 0, 0.25);
			Wait(2);
			MoveForTime(3.0, -1, 0, 0.25);
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {
		stick = new Joystick(0);
		drive->SetSafetyEnabled(false);
	}

	void TeleopPeriodic() {
		while (IsOperatorControl() && IsEnabled()) {
			drive->ArcadeDrive(stick);
			Wait(0.01);
		}
	}

	void TestPeriodic() {
		drive->Drive(0.1, 0.0);
		Wait(0.01);
	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	// (float) distance - Distance forward or backwards, in feet
	// (float) rotation - Rotation right or left, in degrees
	// (float) speed - The speed in which to move (0.0 to 1.0)
	void MoveDistance(float distance, float rotation, float speed) {
		// TO FINISH
	}

	// (float) time - Time to move
	// (integer) move - Weather to move forwards (1), backwards (-1) or stay still (0)
	// (integer) rotate - Weather to rotate right (1), left (-1) or stay still (0)
	// (float) speed - The speed in which to move (0.0 to 1.0)
	void MoveForTime(float time, int move, int rotate, float speed) {
		timer = new Timer();
		float moveSpeed = 0.0;
		float rotateSpeed = 0.0;

		if (move == 1) { moveSpeed = speed; }
		else if (move == -1) { moveSpeed = -speed; }

		if (rotate == 1) { rotateSpeed = speed; }
		else if (rotate == -1) { rotateSpeed = -speed; }

		while(timer->Get() < time) {
			drive->Drive(moveSpeed, rotateSpeed);
			Wait(0.01);
		}
	}
};

START_ROBOT_CLASS(Robot)
