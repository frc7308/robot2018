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
	JoystickButton* stopZeroButton = new JoystickButton(stick, 7);
	JoystickButton* wheelTrigger = new JoystickButton(wheel, 1);
	JoystickButton* rotate180button = new JoystickButton(stick, 6);
	JoystickButton* pickupPresetButton = new JoystickButton(liftStick, 5);
	JoystickButton* switchPresetButton = new JoystickButton(liftStick, 3);
	JoystickButton* scalePresetButton = new JoystickButton(liftStick, 6);

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
		chooser.AddObject("Left -> Switch", "left1");
		chooser.AddObject("Right -> Switch", "right1");
		chooser.AddObject("Left -> Scale/Switch", "left2");
		chooser.AddObject("Right -> Scale/Switch", "right2");
		chooser.AddObject("Forwards", "forwards");
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

		SmartDashboard::PutString("Aligning", "NO");
		SmartDashboard::PutString("Turn180", "NO");
	}

	void DisabledPeriodic() {
		SmartDashboard::PutNumber("jetson", jetson->GetNumber("rotate", 0.0));
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
					MoveUntilAtLiftHeight(0.75, 0.0, 300.0);
					DriveStraightForTime(1.0, 0.75);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(1.0);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.3);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(15);
				} else if(gameData[0] == 'R') {
					MoveUntilAtAngle(0.3, 45, 300.0);
					MoveUntilAtLiftHeight(0.75, 0.0, 300.0);
					DriveStraightForTime(1.0, 0.75);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(1.0);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.3);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(15);
				}
			}
		} else if (autoSelected == "left1") {
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
						MoveUntilAtAngle(-0.3, 110.0, -550.0);
						DriveStraightForTime(0.7, -0.6);
						MoveUntilAtAngle(0.0, 20, -550.0);
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
						Wait(15);
				} else {
					DriveStraightForTime(2.2, 0.7);
					MoveUntilAtAngle(0.5, 90, 0.0);
					DriveStraightForTime(1.9, 0.7);
					Wait(15);
				}
	        }
		} else if (autoSelected == "left2") {
	        if(gameData.length() > 0) {
	        	if (gameData[1] == 'L') {
					MoveForTime(prefs->GetDouble("LiftMoveForwardTime", 3.5), 0.5, 0.0, 1000.0);
					MoveForTime(0.3, 0.2, 0.0, 1000.0);
					MoveUntilAtLiftHeight(0.0, 0.0, 3550.0);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(1.0);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.3);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
					MoveForTime(1.0, 0.25, 0.0, 3550.0);
					MoveUntilAtLiftHeight(0.0, 0.0, 100);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					MoveUntilAtLiftHeight(0.0, 0.0, -550);
					Wait(15);
	        	} else if(gameData[0] == 'L') {
						DriveStraightForTime(1.8, 0.7);
						MoveUntilAtAngle(0.3, 90.0, 300.0);
						MoveUntilAtLiftHeight(0.0, 0.0, 300.0);
						clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
						Wait(0.5);
						clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
						Wait(0.3);
						clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
						Wait(0.2);
						MoveUntilAtAngle(-0.3, 110.0, -550.0);
						DriveStraightForTime(0.7, 0.6);
						MoveUntilAtAngle(0.0, 20, -550.0);
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
						Wait(15);
	        	} else {
	        		DriveStraightForTime(2.2, 0.7);
	        		MoveUntilAtAngle(0.5, 90, 0.0);
	        		DriveStraightForTime(1.9, 0.7);
	        		Wait(15);
	        	}
	        }
		} else if (autoSelected == "right1") {
	        if(gameData.length() > 0) {
				if(gameData[0] == 'R') {
					DriveStraightForTime(1.8, 0.7);
					MoveUntilAtAngle(0.3, -90.0, 300.0);
					MoveUntilAtLiftHeight(0.0, 0.0, 300.0);
					clawSliderSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.5);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
					Wait(0.3);
					clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
					Wait(0.2);
					MoveUntilAtAngle(-0.3, -110.0, -550.0);
					DriveStraightForTime(0.7, -0.6);
					MoveUntilAtAngle(0.0, -20, -550.0);
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
					Wait(15);
				} else {
					DriveStraightForTime(2.2, 0.7);
					MoveUntilAtAngle(0.5, -90, 0.0);
					DriveStraightForTime(1.9, 0.7);
					Wait(15);
				}
			} else if (autoSelected == "forwards") {
				DriveStraightForTime(1.25, 0.75);
				Wait(15);
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
		SmartDashboard::PutNumber("height", liftEncoder->GetDistance());
		SmartDashboard::PutNumber("lift", liftEncoder->Get());
		SmartDashboard::PutNumber("jetson", jetson->GetNumber("rotate", 0.0));

		while (IsOperatorControl() && IsEnabled()) {
			while (!zeroed && (liftEncoder->Get() < 550) && !fakeZeroed) {
				driveLift(0.5);
				if (!liftSwitch->Get()) {
					liftEncoder->Reset();
					zeroed = true;
				}
				if (stopZeroButton->Get()) {
					fakeZeroed = true;
				}
			}
			while (!zeroed && (liftEncoder->Get() > -550) && !fakeZeroed) {
				driveLift(-0.5);
				if (!liftSwitch->Get()) {
					liftEncoder->Reset();
					zeroed = true;
				}
				if (stopZeroButton->Get()) {
					fakeZeroed = true;
				}
			}

			rotateVelocity = wheel->GetY() * 1.2;
			driveVelocity = stick->GetY();

			if (stick->GetTrigger()) {
				alignToClosestBox();
				SmartDashboard::PutString("Aligning", "YES");
			} else {
				SmartDashboard::PutString("Aligning", "NO");
			}

			if (rotate180button->Get()) {
				if (wheel->GetY() > 0.1) {
					turnToAngle(170);
					SmartDashboard::PutString("Turn180", "YES");
				} else if (wheel->GetY() < -0.1) {
					turnToAngle(-170);
					SmartDashboard::PutString("Turn180", "YES");
				} else {
					SmartDashboard::PutString("Turn180", "NO");
				}
			} else {
				SmartDashboard::PutString("Turn180", "NO");
			}

			if (wheelTrigger->Get()) {
				if (wheel->GetY() > 0.1) {
					rotateVelocity = 0.3;
				} else if (wheel->GetY() < -0.1) {
					rotateVelocity = -0.3;
				}
			}

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
				clawSliderSolenoid->Set(frc::DoubleSolenoid::kReverse);
				clawOpen = false;
			}
			if (clawOpen) {
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
			} else {
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kReverse);
			}

			if (throwButton->Get() && clawOut) {
				boxEjectorSolenoid->Set(frc::DoubleSolenoid::kForward);
				clawActuatorSolenoid->Set(frc::DoubleSolenoid::kForward);
			}

			liftVelocity = -1 * liftStick->GetY();

			if (pickupPresetButton->Get()) {
				moveLiftToPosition(-500);
			} else if (switchPresetButton->Get()) {
				moveLiftToPosition(300);
			} else if (scalePresetButton->Get()) {
				moveLiftToPosition(3500);
			} else {
				lifting = false;
			}

			if (liftEncoder->Get() < -400) {
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
	static const float turning_kP = 0.033333;
	static const float lift_kP = 0.005;
	std::string gameData;

	double goal_align;
	bool aligning;

	double goal_lift;
	bool lifting;

	// AUTONOMOUS COMMANDS (blocking)
	void MoveForTime(float time, float movement, float rotation, float lift) {
		timer = new Timer();
		timer->Reset();
		timer->Start();
		while(timer->Get() < time) {
			drivetrain->Drive(-1 * movement, rotation);
			moveLiftToPosition(lift);
			Wait(0.01);
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
		timer->Stop();
	}

	void MoveForTimeWithAngle(float time, float movement, float angle, float lift) {
		timer = new Timer();
		timer->Reset();
		timer->Start();
		float relativeAngle = gyro->GetAngle() + angle;
		while(timer->Get() < time) {
			turnToAngle(angle);
			moveLiftToPosition(lift);
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
		timer->Stop();
	}

	void MoveUntilAtAngle(float movement, float angle, float lift) {
		float relativeAngle = gyro->GetAngle() + angle;
		while(abs(relativeAngle - gyro->GetAngle()) > 3) {
			turnToAngle(angle);
			moveLiftToPosition(lift);
		}
		drivetrain->Drive(0.0, 0.0);
		driveLift(0.0);
	}

	void MoveUntilAtLiftHeight(float movement, float rotation, float lift) {
		while(abs(lift - liftEncoder->Get()) > 25) {
			drivetrain->Drive(-1 * movement, rotation);
			moveLiftToPosition(lift);
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

	// Utility commands

	float clamp(float x, float a, float b) {
	    return x < a ? a : (x > b ? b : x);
	}

	void driveLift(double velocity) {
		liftMotor1->SetSpeed(velocity);
		liftMotor2->SetSpeed(velocity);
		liftMotor3->SetSpeed(velocity);
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

	// Non-blocking movement commands

	void alignToClosestBox() {
		turnToAngle(jetson->GetNumber("rotate", 0.0));
	}

	void turnToAngle(float angle) {
		if (aligning == false) {
			goal_align = gyro->GetAngle() + angle;
			aligning = true;
		}
		if (aligning == true) {
			float offAngle = goal_align - gyro->GetAngle();
			rotateVelocity = offAngle * turning_kP;

			if (abs(offAngle) < 1) {
				aligning = false;
			}
		}
	}

	void moveLiftToPosition(float encoderValue) {
		if (lifting == false) {
			goal_lift = liftEncoder->Get() + encoderValue;
			lifting = true;
		}
		if (lifting == true && zeroed) {
			float offValue = goal_lift - liftEncoder->Get();
			liftVelocity = offValue * lift_kP;

			if (abs(offValue) < 25) {
				lifting = false;
			}
		}
	}
};

// Start robot
START_ROBOT_CLASS(Robot)
