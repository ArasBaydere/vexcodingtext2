/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    1, 2, 3, 4      
// roboHand_MotorGroup  motor_group   5, 6            
// intakeMotor          motor         7               
// Controller1          controller                    
// pneaumtaicSystem     digital_out   A               
// DistanceSensor       distance      8               
// opticalVision        optical       9               
// inertialSensor       inertial      10              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include  <fstream>
using namespace vex;

// A global instance of competition
competition Competition;
int sayac_Default = 0;
int sayac = sayac_Default;
float targetAngle;
bool angleSide;

// define your global instances of motors and other devices here


void turnAngle(float targetAngle, bool angleSide) {
  const float tolerance = 1.0;
  float newAngle = inertialSensor.heading(degrees);
  
  if (angleSide) {
    while (newAngle < targetAngle - tolerance) {
      Drivetrain.turn(right);
      wait(10, msec);
      newAngle = inertialSensor.heading(degrees);
    }
  } else {
    while (newAngle > targetAngle + tolerance) {
      Drivetrain.turn(left);
      wait(10, msec);
      newAngle = inertialSensor.heading(degrees);
    }
  }
  
  Drivetrain.stop();
}

void roboHandAngle(float hedefPozisyon) {
    float mevcutPozisyon = roboHand_MotorGroup.position(degrees); // Mevcut pozisyonu al
    float hata = hedefPozisyon - mevcutPozisyon; // Hata hesapla
    const float tolerans = 1.0; // Tolerans değeri
    int timeOutCounter = 0; // Zaman aşımı sayacı
    const int maxTimeout = 100; // Maksimum zaman aşımı süresi (örneğin, 100 döngü)

    // Hedef pozisyona ulaşana kadar döndür
    while (fabs(hata) > tolerans && timeOutCounter < maxTimeout) {
        if (hata > 0) {
            // Hedef pozisyona gitmek için ileri döndür
            roboHand_MotorGroup.spin(forward, 50, percent); // %50 hızda döner
        } else {
            // Hedef pozisyona gitmek için geri döndür
            roboHand_MotorGroup.spin(reverse, 50, percent); // %50 hızda geri döner
        }

        wait(20, msec); // Kısa bir bekleme süresi
        mevcutPozisyon = roboHand_MotorGroup.position(degrees); // Güncel pozisyonu al
        hata = hedefPozisyon - mevcutPozisyon; // Yeni hatayı hesapla
        timeOutCounter++;
    }

    // Motoru durdur
    roboHand_MotorGroup.stop(); // Hareketi durdur
}



void pneaumtaicSystemOn(){
  pneaumtaicSystem.set(true);

}

void pneaumtaicSystemOff(){
  pneaumtaicSystem.set(false);
}

void logEfficiency() {
    std::ofstream logFile("efficiency_log.txt", std::ios::app); // Log dosyasını aç
    if (logFile.is_open()) {
        for (int i = 0; i < 10; ++i) { // 10 kez verimlilik kaydı
            double efficiency = Drivetrain.efficiency(); // Verimliliği ölç
            logFile << "Verimlilik: " << efficiency << "%" << std::endl; // Log dosyasına yaz
            wait(5, seconds); // 5 saniye bekle
        }
        logFile.close(); // Log dosyasını kapat
    } else {
        // Hata durumu
    }
}


void displayDistance() {
    for (int i = 0; i < 20; ++i) { // 20 kez mesafe ölç
        double distance = DistanceSensor.objectDistance(mm);
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Mesafe: %2.f mm", distance);
        wait(600, msec); // 600 ms bekle
    }
}


void colorDetect(){
  if (opticalVision.isNearObject() && opticalVision.color() == red) {
  opticalVision.setLight(ledState::on);
  //x motor hareketi ile kırmızı halkayı fırlat
}
  else{
    opticalVision.setLight(ledState::off);
  }

}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  Drivetrain.setDriveVelocity(100,percent);
  roboHand_MotorGroup.setVelocity(75,rpm);
  intakeMotor.setVelocity(200,rpm);
  Drivetrain.setTurnVelocity(100,percent);
  roboHand_MotorGroup.setPosition(0,degrees);
  intakeMotor.setPosition(0,degrees);
  inertialSensor.calibrate();
  while(inertialSensor.isCalibrating()){
    wait(100,msec);
  }
  inertialSensor.setHeading(0,degrees);
  inertialSensor.setRotation(0,degrees);

  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  // ..........................................................................
  
  roboHandAngle(50);
  Drivetrain.driveFor(forward,200,mm);
  //
  while(sayac < 5){
    roboHandAngle(0);
    roboHandAngle(20);
    sayac += 1;
  }
  sayac = sayac_Default;
  //
  roboHandAngle(0);
  roboHandAngle(80);
  //
  
  turnAngle(45.0,false);
  Drivetrain.driveFor(forward,250,mm);
  roboHandAngle(60);
  while (sayac <4){
    roboHandAngle(40);
    roboHandAngle(0);
  }
  sayac = sayac_Default;
  //
  roboHandAngle(80);
  Drivetrain.driveFor(reverse,50,mm);
  turnAngle(45.0,true);
  Drivetrain.driveFor(forward,80,mm);
  //
  while(true){
    double robohandDegree;
    robohandDegree = roboHand_MotorGroup.position(degrees);
    if(robohandDegree > 45) {
      roboHand_MotorGroup.setPosition(robohandDegree - 1, degrees);
    } else {
      break;
    }
    wait(20,msec);
  }


  // ..........................................................................
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


int main() {
  
  Controller1.ButtonX.pressed(pneaumtaicSystemOn);
  Controller1.ButtonY.pressed(pneaumtaicSystemOff);

  Drivetrain.setStopping(brake);
  roboHand_MotorGroup.setStopping(brake);
  intakeMotor.setStopping(brake);
  thread distanceThread = thread(displayDistance);
  thread efficiencyThread = thread(logEfficiency);
  thread colordetectThread = thread(colorDetect);
  //
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
