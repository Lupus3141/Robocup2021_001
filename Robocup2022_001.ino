// Test
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servoArm;
Servo servoString;

VL53L0X laserFront;

int motorA1 = 33;
int motorA2 = 34;
int motorApwm = 35;

int motorB1 = 38;
int motorB2 = 37;
int motorBpwm = 14;

int servoAPin = 0;
int servoBPin = 0;

int echoPin = 0;
int triggerPin = 0;

int buzzerPin = 30;
int ledGreenPin = 0;
int ledYellowPin = 1;
int ledRedPin = 2;

int voltageDivider = A2;

int xy = 0;
boolean rescueFlag = false;
int MOTORSPEED = 100;
int SENSITIVITY = 70;

//int neunzigGrad = 840; //delay für "perfekte" 90 Grad Drehung
float origin; // Speichert pos ab, an welcher der Roboter perfekt ausgerichtet ist
String readString;

void setup() {
	Serial.begin(9600);
	Serial2.begin(9600);
	pinMode(24, INPUT_PULLUP);
	pinMode(buzzerPin, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(ledGreenPin, OUTPUT);
	pinMode(ledYellowPin, OUTPUT);
	pinMode(ledRedPin, OUTPUT);
	pinMode(motorA1, OUTPUT);
	pinMode(motorA2, OUTPUT);
	pinMode(motorApwm, OUTPUT);
	pinMode(motorB1, OUTPUT);
	pinMode(motorB2, OUTPUT);
	pinMode(motorBpwm, OUTPUT);

	batteryCheck();
	digitalWrite(13, HIGH);

	Wire.setSDA(8);
	Wire.setSCL(7);
	Wire.begin();

	//Gyrosensor
	if (!bno.begin())  {
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		//while (1);
		beep(1000);
	}

	//Lasersensor
	digitalWrite(24, HIGH);
	laserFront.init();
	laserFront.setTimeout(500);
	laserFront.startContinuous();

	laserFront.init();
	laserFront.setTimeout(500);
	laserFront.startContinuous();
	bno.setExtCrystalUse(true);

	servoArm.attach(23);
	servoArm.write(30); //Arm hoch
	delay(900);	
	servoArm.detach();

	servoString.attach(22);
	servoString.write(180); //Seil locker machen, um Kugel aufzunehmen
	delay(700);
	servoString.detach();
}

void loop() {
	// Empfangen
	readString = "";

	while (Serial2.available()) {
		delay(4);
		if (Serial2.available() > 0) {
			char c = Serial2.read();
			readString += c;
		}
	}

	if (readString != "") {
		Serial.println(readString);
		if (readString == "A") {
			drive(0, 0, 0);
			turnRelative(10);
			turnRelative(180);
			drive(-155, -155, 100);
			drive(0, 0, 0);
			armDown();
			armUp();
			turnRelative(160);
		} if (readString == "L") {
			led(1, 0, 0);
			drive(255, 255, 600);
			turnRelative(-90);
			drive(-255, -255, 200);
			drive(255, 255, 1);
			led(0, 0, 0);   
		} if (readString == "R") {
			led(0, 0, 0);
			drive(255, 255, 600);
			turnRelative(90);
			drive(-255, -255, 200);
			drive(255, 255, 1);
			led(0, 0, 0);
		} if (readString == "D") {
			led(1, 1, 1);
			drive(255, 255, 300);
			turnRelative(180);      
			drive(-255, -255, 500);
			drive(255, 255, 1);
		} if (readString == "S") {
			drive(255, 255, 200);
		} if (readString == "STOP") {
			drive(255, 255, 700);
			drive(0, 0, 0);
			led(1, 0, 0);
			drive(0, 0, 100000);
		} if (readString == "gapR") {
			drive(0, 0, 0);
			beep(50);
			drive(255, -255, 100);
			drive(0, 0, 0);
		} if (readString == "gapL") {
			drive(0, 0, 0);
			beep(50);
			drive(-255, 255, 100);
			drive(0, 0, 0);
		} if (readString == "Rescue") { //Teensy erhält Nachricht vom Raspi und prüft jetzt mit seinem Abstandssensor, ob dort wirklich der Rescuebereich ist
			drive(0, 0, 0);
			led(1, 0, 1);
			if (rescueFlag == false && distanceAvg() < 1300 && distanceAvg() > 300) { //prüft, ob Abstand des Lasersensors stimmen kann
				drive(0, 0, 0);
				led(1, 0, 0);
				Serial2.println(8); //sendet an den Raspi, dass dort wirklich der Rescue ist
				rescue();
			} else {
				drive(0, 0, 0);
				led(0, 0, 1);
				Serial2.println(6);
			}
		} else {
			// Linienverfolgung
			int x = readString.toInt();

			if (x < 200 && x > -200) {

				int motorSpeedL = MOTORSPEED + x * SENSITIVITY;
				int motorSpeedR = MOTORSPEED - x * SENSITIVITY;

				if (getYOrientation() > 15.00) {   
					drive(motorSpeedL * 1.5, motorSpeedR * 1.5, 0); 
				} else if (getYOrientation() < -15.00) {
					drive(motorSpeedL * 0.5, motorSpeedR * 0.5, 0);
				} else {
					drive(motorSpeedL, motorSpeedR, 0);
				}
			}
			obstacle2();
		}
	}
}

void beep(int duration) {
	digitalWrite(buzzerPin, HIGH);
	delay(duration);
	digitalWrite(buzzerPin, LOW);
}

void drive(int left, int right, int duration) {

	if (right > 255) {
		right = 255;
	}
	if (right < -255) {
		right = -255;
	}
	if (left > 255) {
		left = 255;
	}
	if (left < -255) {
		left = -255;
	}

	/*if (right == 0) {
		digitalWrite(motorB1, LOW);
		digitalWrite(motorB2, LOW);
		analogWrite(motorBpwm, 0);
	}

	if (left == 0) {
		digitalWrite(motorA1, LOW);
		digitalWrite(motorA2, LOW);
		analogWrite(motorApwm, 0);
	}

	if (right > 0) {
		digitalWrite(motorB1, LOW);
		digitalWrite(motorB2, HIGH);
	}
	if (right < 0) {
		digitalWrite(motorB1, HIGH);
		digitalWrite(motorB2, LOW);
	}
	if (left > 0) {
		digitalWrite(motorA1, LOW);
		digitalWrite(motorA2, HIGH);
	}
	if (left < 0) {
		digitalWrite(motorA1, HIGH);
		digitalWrite(motorA2, LOW);
	}*/

	digitalWrite(motorA1, left < 0 ? HIGH : LOW);
	digitalWrite(motorA2, left <= 0 ? LOW : HIGH);

	digitalWrite(motorB1, right < 0 ? HIGH : LOW);
	digitalWrite(motorB2, right <= 0 ? LOW : HIGH);

	analogWrite(motorApwm, abs(left));
	analogWrite(motorBpwm, abs(right));

	delay(duration);
}

void turnAbsolute(float pos) {	
	if (pos > getXOrientation()) {
		while (getXOrientation() < pos - 2.0) {
			drive(130, -130, 0);
		}
	} else {
		while (getXOrientation() > pos + 2.0) {
			drive(-130, 130, 0);
		}
	}
	drive(0, 0, 0);
}

void turnRelative(float deg) {
	float startPos = getXOrientation();
	float endPos = startPos + deg - 2; // berechne endpos, aber ziehe 2 Grad ab, weil er immer ein Stück zu weit dreht

	if (deg >= 0.0) {
		//p("startPos: ");
		//p(startPos);
		//p("endPos: ");
		//p(endPos);
		if (startPos >= 0.0 && startPos <=  182) {
			//pln("drehe einfach bis Wunschpos");
			while (getXOrientation() < endPos) {
				drive(130, -130, 0);
			}
		} else {
			if (endPos < 359.9999) {
				//pln("drehe einfach bis Wunschpos2");
				while (getXOrientation() < endPos) {
					drive(130, -130, 0);
				}
			} else {
				//pln("drehe bis 0 und dann bis zur Wunschpos");
				while (getXOrientation() > 1.0) { //drehe bis 0
					drive(130, -130, 0);
				}
				endPos = endPos - 360.0;
				while (getXOrientation() < endPos) {
					drive(130, -130, 0);
				}
			}
		}
		drive(-130, 130, 40);
	} else {
		if (startPos >= 0 && startPos < 182) {
			if (endPos >= 0.0) {
				//pln("drehe einfach bis Wunschposition (leftherum)");
				while (getXOrientation() > endPos) {
					drive(-130, 130, 0);
				}
			} else {
				//pln("drehe bis 0 und dann bis Wunschposition (leftherum)");
				while (getXOrientation() < 359.0) {
					drive(-130, 130, 0);
				}
				endPos = endPos + 360;
				while (getXOrientation() > endPos) {
					drive(-130, 130, 0);
				}
			}
		} else {
			//pln("drehe einfach bis Wunschposition (leftherum)");
			while (getXOrientation() > endPos) {
				drive(-130, 130, 0);
			}
		}
		drive(130, -130, 40);
	}
	drive(0, 0, 0);
}

inline void p(String txt) {
	Serial.print(txt);
}

inline void pln(String txt) {
	Serial.println(txt);
}

void batteryCheck() {
	int val = analogRead(voltageDivider);
	if (val > 845) {
		//voll
		led(1, 0, 0);
	}

	if (val < 845 && val > 824) {
		//mittelvoll
		led(1, 1, 0);
	}

	if (val < 824 && val > 785) {
		//mittel
		led(0, 1, 0);
	}

	if (val < 785 && val > 775) {
		//mittelleer
		led(0, 1, 1);
	}

	if (val < 775) {
		//leer
		led(0, 0, 1);
	}

	if (val < 750) {
		//kritisch
		drive(0, 0, 0);
		led(0, 0, 1);
		beep(1000000);
	}
}

void armDown() {
	servoString.attach(22);
	servoString.write(180); //Seil locker machen, um Kugel aufzunehmen
	delay(700);
	servoString.detach();

	servoArm.attach(23);
	servoArm.write(180); //Arm runter
	delay(900);
	servoArm.detach();
}

void armHalfDown() {
	servoString.attach(22);
	servoString.write(180); //Seil locker machen, um Kugel aufzunehmen
	delay(700);
	servoString.detach();

	servoArm.attach(23);
	servoArm.write(140); //Arm runter
	delay(900);
	servoArm.detach();
}

void armUp() {
	servoString.attach(22);
	servoString.write(0); //Seil fest machen, damit er Kugel nicht verliert
	delay(700);
	servoString.detach();

	servoArm.attach(23);
	servoArm.write(30); //Arm hoch
	delay(900);
	servoArm.detach();
}

int distance() {
	//returns distance of the front laser sensor

	int distance = laserFront.readRangeContinuousMillimeters();
	//Serial.println(distance);
	if (laserFront.timeoutOccurred()) {
		Serial.print("TIMEOUT -> check connections");
	}

	return distance;
}

int distanceAvg() {
	int distanceValues[5];

	int min = 1000000;
	int max = 0;

	int sum = 0;

	for(int i = 0; i < 10; i++) {
		if(i <= 4) {
			distanceValues[i] = distance();
			if(distanceValues[i] < min) {
				min = distanceValues[i];
			}
			sum += distanceValues[i];
		}
		continue;

		if(distanceValues[i - 5] > max) {
			max = distanceValues[i - 5];
		}
	}

	//int avg = (((distanceValues[0] + distanceValues[1] + distanceValues[2] + distanceValues[3] + distanceValues[4]) - min) - max) / 3; //bildet Durchschnitt der mittleren 3 Messungen
	return (sum - min - max) / 3; // average of the three median measurements
}

float getXOrientation() {
	sensors_event_t event;
	bno.getEvent(&event);
	float c = (float)event.orientation.x;
	return c;
}

float getYOrientation() {
	sensors_event_t event;
	bno.getEvent(&event);
	float c = (float)event.orientation.y;
	return c;
}

float getZOrientation() {
	sensors_event_t event;
	bno.getEvent(&event);
	float c = (float)event.orientation.z;
	return c;
}

void debug(bool statement) {
	if (statement) {
		//debug everything

		//laser sensor front:
		Serial.print("Distance: ");
		Serial.print(distance());
		Serial.print("mm");
	}
}

void obstacle2() {
	if (distance() < 50) {
		if (distance() < 60) { //checkt, ob wirklich ein Hindernis erkannt wurde
			if (distance() < 60) {
				drive(0, 0, 0);
				led(1, 1, 1);
				drive(-255, -255, 200);
				turnRelative(50);
				drive(255, 255, 200);
				for (int i = 0; i < 10; i++) {
					drive(255, 255, 110);
					turnRelative(-5);
				}
				drive(0, 0, 500);
				drive(255, 255, 300);
				turnRelative(38);
				drive(-255, -255, 20);
			}
		}
	}
}

void obstacle() {
	if (distance() < 50) {
		if (distance() < 60) { //checkt, ob wirklich ein Hindernis erkannt wurde
			if (distance() < 60) {
				drive(0, 0, 0);
				led(1, 1, 1);
				//Serial2.print("Dose");
				beep(50);
				drive(-150, -150, 200);
				//drive(255, -255, neunzigGrad / 1.3);
				turnRelative(70);
				drive(255, 255, 100);
				int x = 200;
				Serial2.flush();
				beep(50);
				drive(0, 0, 0);
				delay(1000);
				led(0, 0, 0);
				/*while (Serial2.available()) { //hängt sich in dieser loop auf Fehlercode: jdhjkdfhg
					delay(4);
					Serial.read();
					digitalWrite(ledRedPin, HIGH);
					}*/
				digitalWrite(ledRedPin, LOW);

				bool a = false;

				while (abs(x) == 0 || abs(x) > 3) {
					readString = "";

					while (Serial2.available()) {
						delay(4);
						if (Serial2.available() > 0) {
							char c = Serial2.read();
							readString += c;
						}
					}

					if (readString != "") {
						x = readString.toInt();
					} else {
						x = 0;
					}
					drive(30, 255, 0);
					a = -a;
					delay(20);
				}
				led(0, 1, 0);
				drive(255, 255, 300);
				drive(255, -255, 500);
				drive(255, 255, 100);
				drive(0, 0, 0);
				beep(50);
			}
			led(0, 0, 0);
		}
	}
}
/*
void ausrichten() {
	bool ungerade = true;
	int last = distanceAvg();
	drive(150, -150, 200);
	drive(0, 0, 0);
	int current = distanceAvg();

	if (current < last) {
		while (ungerade) {
			led(1, 1, 1);
			last = distanceAvg();
			drive(150, -150, 50);
			drive(0, 0, 0);
			current = distanceAvg();
			delay(100);

			Serial.print("last: ");
			Serial.print(last);
			Serial.print("  ");
			Serial.print("current: ");
			Serial.print(current);
			Serial.print("  ");

			if (current > last) {
				//ist gerade
				beep(50);
				drive(0, 0, 0);
				led(0, 0, 0);
				drive(-255, -255, 3000);
				drive(0, 0, 0);
				delay(100000);
				//ungerade = false;
				//break;
			}
		}
	}
}
*/
void rescue() {
	drive(0, 0, 0);
	beep(20);
	boolean rescue = true;
	while (rescue) {
		if (Serial2.available() > 0) {
			String incomingString = Serial2.readString();
			//incomingString.trim();
			String xval = getValue(incomingString, ':', 0);
			String yval = getValue(incomingString, ':', 1);
			String zval = getValue(incomingString, ':', 2);
			int motorleft = xval.toInt();
			int motorright = yval.toInt();
			int duration = zval.toInt();   
			pln(incomingString);

			if (incomingString == "armUp") {
				drive(0, 0, 0);
				armUp();
				Serial2.println(1);
			} else if (incomingString == "armDown") {
				drive(0, 0, 0);
				armDown();
				Serial2.println(1);
			} else if (incomingString == "setOrigin") {
				origin = getXOrientation();
				beep(50);
				p("origin set to: ");
				p(origin);
				pln("");
				Serial2.println(1);
			} else if (incomingString == "turnToOrigin") {
				turnAbsolute(origin);
				Serial2.println(1);
			} else if (incomingString == "exit") {
				rescueFlag == true;
				drive(0, 0, 0);
				Serial2.println(1);
				drive(0, 0, 3000);
				return;
			} else if (incomingString == "driveToBlackCornerAndSaveVictim") {
				turnRelative(90);
				while (distanceAvg() > 100) {
					drive(255, 255, 0);
					led(0, 0, 1);
				}
				drive(255, 255, 700);
				drive(-255, -255, 500);
				turnRelative(90);
				while (distanceAvg() > 130) {
					drive(255, 255, 0);
					led(0, 0, 1);
				}
				drive(0, 0, 0);
				turnRelative(90);
				while (distanceAvg() > 400) {
					drive(255, 255, 0);
				}
				drive(0, 0, 0);
				turnRelative(45);
				drive(255, 255, 650);
				turnRelative(90);
				drive(-255, -255, 500);
				drive(0, 0, 500);
				armHalfDown();
				armUp();
				drive(255, 255, 1500);
				turnAbsolute(origin);
				drive(255, 255, 800);
				turnRelative(90);
				while (distanceAvg() > 100) {
					drive(255, 255, 0);
					led(0, 0, 1);
				}
				drive(255, 255, 700);
				drive(-255, -255, 300);
				turnRelative(-90);
				drive(0, 0, 0);
				Serial2.println(1);
			} else {
				/*
				p(motorleft);
				p("  ");
				p(motorright);
				pln("");
				*/
				if (motorleft == 0 && motorright == 0) {
					if (duration == 0) { //drehe dich zum Ursprung

					} else {
						turnRelative(duration);
						Serial2.println(1);
					}
				} else {
					drive(motorleft, motorright, duration);
					drive(0, 0, 0);
					Serial2.println(1);
				}
			}
		}
	}
}

String getValue(String data, char separator, int index) { //returns ints from mutliple Strings seperated by a character
	int found = 0;
	int strIndex[] = { 0, -1 };
	int maxIndex = data.length() - 1;

	for (int i = 0; i <= maxIndex && found <= index; i++) {
			if (data.charAt(i) == separator || i == maxIndex) {
					found++;
					strIndex[0] = strIndex[1] + 1;
					strIndex[1] = (i == maxIndex) ? i+1 : i;
			}
	}
	return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void led(int green, int yellow, int red) {
	/*if (red > 1) {
		red = 1;
	}
	if (red < 0) {
		red = 0;
	}

	if (green > 1) {
		green = 1;
	}
	if (green < 0) {
		green = 0;
	}

	if (yellow > 1) {
		yellow = 1;
	}
	if (yellow < 0) {
		yellow = 0;
	}*/
	digitalWrite(ledGreenPin, constrain(green, 0, 1));
	digitalWrite(ledYellowPin, constrain(yellow, 0, 1));
	digitalWrite(ledRedPin, constrain(red, 0, 1));
}
