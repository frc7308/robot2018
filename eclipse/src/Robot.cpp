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
#include <DigitalInput.h>
#include <WPILib.h>

#include <NetworkTables/NetworkTable.h>

class Robot : public frc::IterativeRobot {
public:
	Joystick *stick = new Joystick(0);
	Joystick *wheel = new Joystick(1);
	Joystick *liftStick = new Joystick(3);

	RobotDrive *drivetrain = new RobotDrive(0, 1, 2, 3);

	Timer *timer;

	Spark *liftMotor1 = new Spark(4);
	Spark *liftMotor2 = new Spark(5);
	Spark *liftMotor3 = new Spark(6);

	DoubleSolenoid *boxEjectorSolenoid = new DoubleSolenoid(0, 1);
	frc::Solenoid *clawSliderSolenoid = new Solenoid(2);
	frc::Solenoid *clawActuatorSolenoid = new Solenoid(3);
	Compressor *compressor = new Compressor(0);

	Encoder *liftEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);

	JoystickButton* button1 = new JoystickButton(stick, 4);
	JoystickButton* button2 = new JoystickButton(stick, 6);

//	DigitalInput liftSwitch = new DigitalInput(6);

//	NetworkTable *jetson = NetworkTable::GetTable("jetson");

	void DisabledInit() {
		wheel->SetYChannel(0);

		liftEncoder->SetMinRate(10);
		liftEncoder->SetDistancePerPulse(5);
		liftEncoder->SetSamplesToAverage(7);
		liftEncoder->Reset();

		/*drivetrain->SetSafetyEnabled(safety);
		liftMotor1->SetSafetyEnabled(safety);
		liftMotor2->SetSafetyEnabled(safety);
		liftMotor3->SetSafetyEnabled(safety);*/
	}

	void DisabledPeriodic() {
		// Zero
	}

	void AutonomousInit() override {
		compressor->SetClosedLoopControl(true);

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

			driveVelocity = clamp(driveVelocity, -1.0, 1.0);
			rotateVelocity = clamp(rotateVelocity, -1.0, 1.0) * -1;

			drivetrain->ArcadeDrive(driveVelocity, rotateVelocity);
			Wait(0.01);
/*
			if (stick->GetTrigger()) {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
			} else {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
			}*/
			if (button1->Get()) {
				clawActuatorSolenoid->Set(true);
			} else {
				clawActuatorSolenoid->Set(false);
			}
			if (button2->Get()) {
				clawSliderSolenoid->Set(true);
			} else {
				clawSliderSolenoid->Set(false);
			}

			liftMotor1->SetSpeed(liftStick->GetY());
			liftMotor2->SetSpeed(liftStick->GetY());
			liftMotor3->SetSpeed(liftStick->GetY());
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
	bool safety = false;

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
