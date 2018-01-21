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

		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			MoveForTime(2, 0.0, 0.5);
			Wait(2);
			MoveForTime(2, 0.0, 0.5);
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
		drive = new RobotDrive(0, 1, 2, 3);
		stick = new Joystick(0);
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
	// (float) movement - The speed in which to move forwards or backwards (0.0 to 1.0)
	// (float) rotation - The speed in which to rotate left or right (0.0 to 1.0)
	void MoveForTime(float time, float movement, float rotation) {
		timer = new Timer();
		while(timer->Get() < time) {
			drive->Drive(movement, rotation);
			Wait(0.01);
		}
	}
};

START_ROBOT_CLASS(Robot)
