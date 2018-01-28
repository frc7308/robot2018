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
	frc::DoubleSolenoid clawActuationSolenoid {1, 2};
	Encoder *clawEncoder;

	void DisabledInit() {
		// Zero
	}

	void DisabledPeriodic() {
		// Zero
	}

	void AutonomousInit() override {
		drive = new RobotDrive(0, 1, 2, 3);
		clawEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);

		clawEncoder->SetMinRate(10);
		clawEncoder->SetDistancePerPulse(5);
		clawEncoder->SetSamplesToAverage(7);
		clawEncoder->Reset();

		m_autoSelected = m_chooser.GetSelected();
		// Uncomment below to use SmartDashboard
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto init goes here
		} else {
			// Default Auto init goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			//clawActuationSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
			clawActuationSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
			//clawActuationSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
			drive->Drive(1.0, 0.0);
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

	// Component variables. State 0 is default when not zeroed.
	float liftAngle; // Angle from lift encoder converted to 4320 deg
	float goal_liftAngle; // Goal for the lift angle
	float liftHeight; // Calculated lift height based on angle
	float goal_liftHeight; // Goal for the lift height
	int liftState; // 0 - lowest, 1 - switch, 2 - scale, -1 - err
	float clawAngle; // Angle from claw encoder around 360 deg
	float goal_clawAngle; // Goal for the claw angle
	int clawState; // 0 - back, 1 - extended claw closed, 2 - extended claw open, -1 - err
	bool robotError; // Is true if an inconsistency or error has been found]

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
