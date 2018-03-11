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
#include <SmartDashboard/SendableChooser.h>
#include <WPILib.h>
#include <Commands/Command.h>

#include <NetworkTables/NetworkTable.h>

class Robot : public frc::IterativeRobot {
public:
	Joystick *stick = new Joystick(0);
	Joystick *wheel = new Joystick(1);
	Joystick *bboard = new Joystick(2);
	Joystick *liftStick = new Joystick(3);

	RobotDrive *drivetrain = new RobotDrive(0, 1, 2, 3);

	Timer *timer;
	Timer *clawTimer;

	Spark *liftMotor1 = new Spark(4);
	Spark *liftMotor2 = new Spark(5);
	Spark *liftMotor3 = new Spark(6);

	DoubleSolenoid *boxEjectorSolenoid = new DoubleSolenoid(0, 1);
	DoubleSolenoid *clawSliderSolenoid = new DoubleSolenoid(2, 3);
	DoubleSolenoid *clawActuatorSolenoid = new DoubleSolenoid(4, 5);
	Compressor *compressor = new Compressor(0);

	Encoder *liftEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k2X);
	AnalogGyro *gyro;

	JoystickButton* button1 = new JoystickButton(bboard, 2);
	JoystickButton* button2 = new JoystickButton(bboard, 1);
	JoystickButton* button3 = new JoystickButton(bboard, 4);
	JoystickButton* throwButton = new JoystickButton(bboard, 3);

	DigitalInput *liftSwitch = new DigitalInput(3);

	std::shared_ptr<NetworkTable> jetson = NetworkTable::GetTable("jetson");

	frc::SendableChooser<std::string> chooser;
	frc::SendableChooser<std::string> testChooser;
	Preferences *prefs;

	void RobotInit() {
		compressor->SetClosedLoopControl(true);

		liftEncoder->SetDistancePerPulse(0.0156862745); // 4 / 255;
		drivetrain->SetExpiration(0.1);

		gyro(0);
		gyro->Calibrate();

		wheel->SetYChannel(0);

		chooser.AddDefault("Middle -> Switch", "middle");
		chooser.AddObject("Left -> Switch/Scale", "left");
		chooser.AddObject("Right -> Switch/Scale", "right");
		chooser.AddObject("NONE", "none");
		SmartDashboard::PutData("autonomous", &chooser);

		testChooser.AddDefault("NONE", "none");
		testChooser.AddObject("MoveForTime", "movefortime");
		testChooser.AddObject("DriveStraightForTime", "drivestraightfortime");
		testChooser.AddObject("MoveForTimeWithAngle", "movefortimewithangle");
		testChooser.AddObject("MoveUntilAtAngle", "moveuntilatangle");
		testChooser.AddObject("MoveUntilAtLiftHeight", "moveuntilatliftheight");
		testChooser.AddObject("DANGEROUS - Unchecked Control", "control");
		testChooser.AddObject("Test", "test");
		SmartDashboard::PutData("testing mode", &testChooser);

		CameraServer::GetInstance()->StartAutomaticCapture();

		prefs = Preferences::GetInstance();

		compressor->SetClosedLoopControl(true);
		drivetrain->SetSafetyEnabled(false);
	}

	void DisabledPeriodic() {
		SmartDashboard::PutString("jetson", jetson->GetString("rotate", "nope"));
		SmartDashboard::PutNumber("lift height", liftEncoder->GetDistance());
		SmartDashboard::PutNumber("At zero", liftSwitch->Get());
		if (!liftSwitch->Get()) {
			liftEncoder->Reset();
		}
	}

	void AutonomousInit() override {
		gyro->Reset();

		autoSelected = chooser.GetSelected();

		if (autoSelected == "middle") {
			// Custom Auto init goes here
		}
		compressor->SetClosedLoopControl(true);
		drivetrain->SetSafetyEnabled(false);

		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}

	void AutonomousPeriodic() {
		clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
		clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
		if (!liftSwitch->Get()) {
			zeroed = true;
			fakeZeroed = false;
			liftEncoder->Reset();
		}
		if (autoSelected == "middle") {
			if(gameData.length() > 0) {
				if(gameData[0] == 'L') {
					MoveUntilAtAngle(0.3, -45, 300.0);
					DriveStraightForTime(0.7, 0.5);
				}
			}
		} else if (autoSelected == "left") {
	        if(gameData.length() > 0) {
	        	if(gameData[0] == 'L') {
					DriveStraightForTime(1.8, 0.7);
					MoveUntilAtAngle(0.3, 90.0, 300.0);
					MoveUntilAtLiftHeight(0.0, 0.0, 300.0);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.5);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.2);
					MoveUntilAtAngle(-0.3, 110.0, -600.0);
					DriveStraightForTime(0.7, 0.6);
					MoveUntilAtAngle(0.0, 20, -600.0);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					DriveStraightForTime(0.2, 0.25);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					MoveUntilAtLiftHeight(0.0, 0.0, 300.0);
					DriveStraightForTime(0.3, 0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					boxEjectorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.2);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(100);
	        	} else if (gameData[1] == 'L') {
					MoveForTime(prefs->GetDouble("LiftMoveForwardTime", 3.5), 0.5, 0.0, 1000.0);
					MoveForTime(0.3, 0.2, 0.0, 1000.0);
					MoveUntilAtLiftHeight(0.0, 0.0, 3600.0);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(1.0);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.3);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
					MoveForTime(1.0, 0.25, 0.0, 3600.0);
					MoveUntilAtLiftHeight(0.0, 0.0, 100);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					MoveUntilAtLiftHeight(0.0, 0.0, -600);
					Wait(100);
	        	} else {
	        		DriveStraightForTime(2.2, 0.7);
	        		MoveUntilAtAngle(0.5, -90, 0.0);
	        		DriveStraightForTime(1.9, 0.7);
	        		MoveUntilAtAngle(0.0, -90, 0.0);
	        	}
	        }
		} else if (autoSelected == "right") {
			if(gameData.length() > 0) {
				if(gameData[0] == 'R') {
					MoveForTime(0.2, 0.5, -0.07, 0.0);
					MoveForTime(2.1, 0.5, 0.0, 0.0);
					MoveForTime(0.2, 0.0, 0.0, 0.0);
					MoveForTime(1.9, 0.5, 0.6, 300.0);
					MoveForTime(0.3, 0.25, 0.0, 0.0);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					boxEjectorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.3);
					MoveForTime(0.5, -0.5, 0.0, 0.0);
					MoveForTime(0.85, -0.5, -0.5, -600);
					MoveForTime(2.5, -0.5, 0.005, 0.0);
					MoveForTime(1.0, 0.0, 0.0, 0.0);
					Wait(100);
				} else {
					MoveForTime(3.7, 0.5, 0.0, 0.0);
					MoveForTime(1.0, 0.5, 0.5, 0.0);
					MoveForTime(2.0, 0.5, 0.0, 0.0);
					MoveForTime(0.975, 0.5, 0.5, 0.0);
					MoveForTime(0.1, 0.0, 0.0, 0.0);
					Wait(100);
				}
			} else if (autoSelected == "test") {
				MoveForTime(3.7, 0.5, 0.0, 0.0);
			}
		}
	}

	void TeleopInit() {
		clawOut = true;
		clawOpen = false;
		compressor->SetClosedLoopControl(true);
		drivetrain->SetSafetyEnabled(false);
	}

	void TeleopPeriodic() {
		while (IsOperatorControl() && IsEnabled()) {
			while (!zeroed && (liftEncoder->Get() < 600) && !fakeZeroed) {
				driveLift(0.5);
				if (!liftSwitch->Get()) {
					liftEncoder->Reset();
					zeroed = true;
				}
				if (stick->GetTrigger()) {
					fakeZeroed = true;
				}
			}
			while (!zeroed && (liftEncoder->Get() > -600) && !fakeZeroed) {
				driveLift(-0.5);
				if (!liftSwitch->Get()) {
					liftEncoder->Reset();
					zeroed = true;
				}
				if (stick->GetTrigger()) {
					fakeZeroed = true;
				}
			}
			SmartDashboard::PutNumber("height", liftEncoder->GetDistance());
			SmartDashboard::PutNumber("encoder val", liftEncoder->Get());
			if (accurateTurn) {
				rotateVelocity = wheel->GetY() * 0.8;
			} else {
				rotateVelocity = wheel->GetY() * 1.2;
			}
			driveVelocity = stick->GetY();

			if (liftEncoder->Get() >= 200) {
				speedLimit = 100 / liftEncoder->Get() + 400;
			} else {
				speedLimit = 1.0;
			}

			driveVelocity = clamp(driveVelocity, -1 * speedLimit, speedLimit);
			rotateVelocity = clamp(rotateVelocity, -1 * speedLimit, speedLimit) * -1;

			drivetrain->ArcadeDrive(driveVelocity, rotateVelocity);
			Wait(0.01);

			if (button1->Get()) {
				clawOut = true;
			}
			if (button2->Get() && liftEncoder->Get() > 100) {
				clawOut = false;
			}
			if (button3->Get()) {
				clawOpen = true;
			} else {
				clawOpen = false;
			}

			if (liftStick->GetTrigger()) {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kReverse);
			}
			if (clawOut) {
				clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
				/*
				clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
				clawOpen = false;
				*/
			}
			if (clawOpen) {
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
			}

			if (throwButton->Get()) {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
			}

			liftVelocity = -1 * liftStick->GetY();
			if (liftEncoder->Get() < 0) {
				liftVelocity = clamp(liftVelocity, -0.6, 1.0);
			}
			if (liftAtTop && !fakeZeroed) {
				liftVelocity = clamp(liftVelocity, -1.0, 0.0);
			}
			if (liftAtBottom && !fakeZeroed) {
				liftVelocity = clamp(liftVelocity, 0.0, 1.0);
			} else if (liftEncoder->Get() < -260 && !fakeZeroed) {
				liftVelocity = clamp(liftVelocity, -0.4, 1.0);
			}
			driveLift(liftVelocity);

			if (!liftSwitch->Get()) {
				liftEncoder->Reset();
				zeroed = true;
				fakeZeroed = false;
			}
			SmartDashboard::PutNumber("At zero", liftSwitch->Get());
			if (liftEncoder->Get() > 3600) {
				liftAtTop = true;
			} else {
				liftAtTop = false;
			}
			if ((liftEncoder->Get() < 100 && !clawOut) || (liftEncoder->Get() < -650 && clawOut)) {
				liftAtBottom = true;
			} else {
				liftAtBottom = false;
			}
		}
	}

	void TestInit() {
		drivetrain->SetSafetyEnabled(false);
		testSelected = testChooser.GetSelected();
	}

	void TestPeriodic() {
		SmartDashboard::PutNumber("height", liftEncoder->GetDistance());
		SmartDashboard::PutNumber("encoder val", liftEncoder->Get());
		if (testSelected == "movefortime") {
			MoveForTime(2.0, 0.5, 0.0, 200);
			Wait(600);
		} else if (testSelected == "drivestraightfortime") {
			DriveStraightForTime(2.0, 0.5);
			Wait(600);
		} else if (testSelected == "movefortimewithangle") {
			MoveForTimeWithAngle(2.0, 0.5, 45, 200);
			Wait(600);
		} else if (testSelected == "moveuntilatangle") {
			DriveStraightForTime(2.0, 0.5);
			Wait(600);
		} else if (testSelected == "moveuntilatliftheight") {
			MoveUntilAtLiftHeight(0.5, 0.0, 400);
			Wait(600);
		} else if (testSelected == "control") {
			if (accurateTurn) {
				rotateVelocity = wheel->GetY() * 0.8;
			} else {
				rotateVelocity = wheel->GetY() * 1.2;
			}
			driveVelocity = stick->GetY();

			driveVelocity = clamp(driveVelocity, -1 * 0.5, 0.5);
			rotateVelocity = clamp(rotateVelocity, -1 * 0.5, 0.5) * -1;

			drivetrain->ArcadeDrive(driveVelocity, rotateVelocity);
			Wait(0.01);

			if (button1->Get()) {
				clawOut = true;
			}
			if (button2->Get() && liftEncoder->Get() > 100) {
				clawOut = false;
			}
			if (button3->Get()) {
				clawOpen = true;
			} else {
				clawOpen = false;
			}

			if (liftStick->GetTrigger()) {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kReverse);
			}
			if (clawOut) {
				clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
			}
			if (clawOpen) {
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
			}

			if (throwButton->Get()) {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
			}

			liftVelocity = (-1 * liftStick->GetY()) / 2;
			driveLift(liftVelocity);

			SmartDashboard::PutNumber("At zero", liftSwitch->Get());
		} else if (testSelected == "test") {

		}
	}

private:
	std::string autoSelected;
	std::string testSelected;

	// Component variables.
	double driveVelocity = 0.0;
	double rotateVelocity = 0.0;
	bool accurateTurn = false;
	bool safety = false;
	bool clawOut = false;
	bool clawOpen = false;
	bool liftAtTop = false;
	bool liftAtBottom = false;
	double liftVelocity = 0.0;
	float speedLimit = 1.0;
	bool zeroed = false;
	bool clawOutReal = false;
	bool fakeZeroed = false;
	static const float straight_kP = 0.03;
	static const float turning_kP = 0.03;
	std::string gameData;

	// (float) time - Time to move
	// (float) movement - The speed in which to move forwards or backwards (0.0 to 1.0)
	// (float) rotation - The speed in which to rotate left or right (0.0 to 1.0)
	void MoveForTime(float time, float movement, float rotation, float lift) {
		timer = new Timer();
		timer->Reset();
		timer->Start();
		while(timer->Get() < time) {
			drivetrain->Drive(-1 * movement, rotation);
			if (abs(lift - liftEncoder->Get()) > 25) {
				if (lift - liftEncoder->Get() > 50) {
					driveLift(1.0);
				} else if (lift - liftEncoder->Get() > 0) {
					driveLift(0.3);
				} else if (lift - liftEncoder->Get() < 50) {
					driveLift(-1.0);
				} else if (lift - liftEncoder->Get() < 0) {
					driveLift(-0.3);
				}
			}
			Wait(0.01);
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
		timer->Stop();
	}

	void MoveForTimeWithAngle(float time, float movement, float angle, float lift) {
		float rotationSpeed = 0.0;
		if (angle > 0) {
			rotationSpeed = 0.8;
		} else if (angle < 0) {
			rotationSpeed = -0.8;
		}
		timer = new Timer();
		timer->Reset();
		timer->Start();
		float relativeAngle = gyro->GetAngle() + angle;
		while(timer->Get() < time) {
			if (abs(relativeAngle - gyro->GetAngle()) < 10) {
				float offAngle = (relativeAngle - gyro->GetAngle()) * -1;
				drivetrain->Drive(-1 * movement, -offAngle * straight_kP);
				Wait(0.004);
			} else {
				drivetrain->Drive(-1 * movement, rotationSpeed);
			}
			if (abs(lift - liftEncoder->Get()) > 25) {
				if (lift - liftEncoder->Get() > 50) {
					driveLift(1.0);
				} else if (lift - liftEncoder->Get() > 0) {
					driveLift(0.3);
				} else if (lift - liftEncoder->Get() < 50) {
					driveLift(-1.0);
				} else if (lift - liftEncoder->Get() < 0) {
					driveLift(-0.3);
				}
			}
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
		timer->Stop();
	}

	void MoveUntilAtAngle(float movement, float angle, float lift) {
		float rotationSpeed = 0.0;
		if (angle > 0) {
			rotationSpeed = 0.8;
		} else if (angle < 0) {
			rotationSpeed = -0.8;
		}
		float relativeAngle = gyro->GetAngle() + angle;
		while(abs(relativeAngle - gyro->GetAngle()) > 5) {
			if (abs(relativeAngle - gyro->GetAngle()) < 10) {
				float offAngle = (relativeAngle - gyro->GetAngle()) * -1;
				drivetrain->Drive(-1 * movement, -offAngle * straight_kP);
				Wait(0.004);
			} else {
				drivetrain->Drive(-1 * movement, rotationSpeed);
			}
			if (abs(lift - liftEncoder->Get()) > 25) {
				if (lift - liftEncoder->Get() > 50) {
					driveLift(1.0);
				} else if (lift - liftEncoder->Get() > 0) {
					driveLift(0.3);
				} else if (lift - liftEncoder->Get() < 50) {
					driveLift(-1.0);
				} else if (lift - liftEncoder->Get() < 0) {
					driveLift(-0.3);
				}
			}
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
	}

	void MoveUntilAtLiftHeight(float movement, float rotation, float lift) {
		while(abs(lift - liftEncoder->Get()) > 25) {
			drivetrain->Drive(-1 * movement, rotation);
			if (lift - liftEncoder->Get() > 50) {
				driveLift(1.0);
			} else if (lift - liftEncoder->Get() > 0) {
				driveLift(0.3);
			} else if (lift - liftEncoder->Get() < 50) {
				driveLift(-1.0);
			} else if (lift - liftEncoder->Get() < 0) {
				driveLift(-0.3);
			}
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
	}

	// Will drive straight while correcting it's rotation using the provided offset angle
	void DriveStraightForTime(float time, float speed) {
		float startAngle = gyro->GetAngle();
		timer = new Timer();
		timer->Reset();
		timer->Start();
		while(timer->Get() < time) {
			float offAngle = (startAngle - gyro->GetAngle()) * -1;
			drivetrain->Drive(-1 * speed, -offAngle * straight_kP);
			Wait(0.004);
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
		timer->Stop();
	}

	float clamp(float x, float a, float b) {
	    return x < a ? a : (x > b ? b : x);
	}

	void driveLift(double velocity) {
		/*liftMotor1->SetSpeed(velocity);
		liftMotor2->SetSpeed(velocity);
		liftMotor3->SetSpeed(velocity);*/
	}

	void findZero(double range) {
		while (liftEncoder->Get() < range && !zeroed) {
			if (!liftSwitch->Get()) {
				liftEncoder->Reset();
				zeroed = true;
				return;
			}
			driveLift(0.2);
		}
		while (liftEncoder->Get() > -1 * range && !zeroed) {
			if (!liftSwitch->Get()) {
				liftEncoder->Reset();
				zeroed = true;
				return;
			}
			driveLift(-0.2);
		}
	}
};

START_ROBOT_CLASS(Robot)
