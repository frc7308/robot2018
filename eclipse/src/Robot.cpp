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
#include <DoubleSolenoid.h>
#include <Solenoid.h>
#include <Compressor.h>
#include <Encoder.h>
#include <Spark.h>

#include <NetworkTables/NetworkTable.h>

class Robot : public frc::IterativeRobot {
public:
	Joystick *stick = new Joystick(0);
	Joystick *wheel = new Joystick(1);
	Joystick *buttons = new Joystick(2);
	Joystick *liftStick = new Joystick(3);

	RobotDrive *drivetrain = new RobotDrive(0, 1, 2, 3);

	Timer *timer;

	Spark *liftMotor1 = new Spark(4);
	Spark *liftMotor2 = new Spark(5);

	DoubleSolenoid *clawActuationSolenoid {1, 2};
	Solenoid *clawPusherSolenoid {1, 2};
	Compressor *compressor = new Compressor(0);

	Encoder *liftEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);

	NetworkTable *jetson = NetworkTable::GetTable("jetson");

	void DisabledInit() {
		wheel->SetYChannel(0);
	}

	void DisabledPeriodic() {
		// Zero
	}

	void AutonomousInit() override {
		compressor->SetClosedLoopControl(true);

		liftEncoder->SetMinRate(10);
		liftEncoder->SetDistancePerPulse(5);
		liftEncoder->SetSamplesToAverage(7);
		liftEncoder->Reset();

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
			clawActuationSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
			liftMotor1->Set(0.5);
			liftMotor2->Set(0.5);
		}
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
		while (IsOperatorControl() && IsEnabled()) {
			if (fullAutonomous) {
				drivetrain->Drive(0.5, 1.0);
			} else {
				if (accurateTurn) {
					rotateVelocity = wheel->GetY() * 0.8;
				} else {
					rotateVelocity = wheel->GetY() * 1.2;
				}
			}
			driveVelocity = stick->GetY();

			driveVelocity = clamp(driveVelocity, 0.0, 1.0);
			rotateVelocity = clamp(rotateVelocity, 0.0, 1.0);

			drivetrain->ArcadeDrive(driveVelocity, rotateVelocity);
			Wait(0.01);
		}
	}

	void TestPeriodic() {
		drivetrain->Drive(0.1, 0.0);
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
	bool robotError; // Is true if an inconsistency or error has been found
	bool fullAutonomous = false;
	double driveVelocity = 0.0;
	double rotateVelocity = 0.0;
	bool accurateTurn = false;

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
			drivetrain->Drive(movement, rotation);
			Wait(0.01);
		}
	}

	float clamp(float x, float a, float b)
	{
	    return x < a ? a : (x > b ? b : x);
	}
};

START_ROBOT_CLASS(Robot)
