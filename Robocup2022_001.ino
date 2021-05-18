/*to do:
	- hängt sich bei while Schleife auf (Fehlercode: jdhjkdfhg)
*/

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servoGreifer;
Servo servoSeil;

VL53L0X laserVorne;

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
int ledgruenPin = 0;
int ledgelbPin = 1;
int ledrotPin = 2;

int voltageDividor = A2;

int xy = 0;
boolean rescueFlag = false;
int MOTORSPEED = 100;
int MULTIPLIKATOR = 70;

int neunzigGrad = 840; //delay für "perfekte" 90 Grad Drehung
float ursprung; // Speichert pos ab, an welcher der Roboter perfekt ausgerichtet ist
String readString;

void setup() {
	Serial.begin(9600);
	Serial2.begin(9600);
	pinMode(24, INPUT_PULLUP);
	pinMode(buzzerPin, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(ledgruenPin, OUTPUT);
	pinMode(ledgelbPin, OUTPUT);
	pinMode(ledrotPin, OUTPUT);
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
	laserVorne.init();
	laserVorne.setTimeout(500);
	laserVorne.startContinuous();


	laserVorne.init();
	laserVorne.setTimeout(500);
	laserVorne.startContinuous();
	bno.setExtCrystalUse(true);

	servoGreifer.attach(23);
	servoGreifer.write(30); //Arm hoch
	delay(900);	
	servoGreifer.detach();

	servoSeil.attach(22);
	servoSeil.write(180); //Seil locker machen, um Kugel aufzunehmen
	delay(700);
	servoSeil.detach();
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
			fahre(0, 0, 0);
			drehe(10);
			drehe(180);
			fahre(-155, -155, 100);
			fahre(0, 0, 0);
			greiferRunter();
			greiferHoch();
			drehe(160);
		} if (readString == "L") {
			led(1, 0, 0);
			fahre(255, 255, 600);
			drehe(-90);
			fahre(-255, -255, 200);
			fahre(255, 255, 1);
			led(0, 0, 0);   
		} if (readString == "R") {
			led(0, 0, 0);
			fahre(255, 255, 600);
			drehe(90);
			fahre(-255, -255, 200);
			fahre(255, 255, 1);
			led(0, 0, 0);
		} if (readString == "D") {
			led(1, 1, 1);
			fahre(255, 255, 300);
			drehe(180);      
			fahre(-255, -255, 500);
			fahre(255, 255, 1);
		} if (readString == "S") {
			fahre(255, 255, 200);
		} if (readString == "STOP") {
			fahre(255, 255, 700);
			fahre(0, 0, 0);
			led(1, 0, 0);
			fahre(0, 0, 100000);
		} if (readString == "gapR") {
			fahre(0, 0, 0);
			beep(50);
			fahre(255, -255, 100);
			fahre(0, 0, 0);
		} if (readString == "gapL") {
			fahre(0, 0, 0);
			beep(50);
			fahre(-255, 255, 100);
			fahre(0, 0, 0);
		} if (readString == "Rescue") { //Teensy erhält Nachricht vom Raspi und prüft jetzt mit seinem Abstandssensor, ob dort wirklich der Rescuebereich ist
			fahre(0, 0, 0);
			led(1, 0, 1);
			if (rescueFlag == false && distanceAvg() < 1300 && distanceAvg() > 300) { //prüft, ob Abstand des Lasersensors stimmen kann
				fahre(0, 0, 0);
				led(1, 0, 0);
				Serial2.println(8); //sendet an den Raspi, dass dort wirklich der Rescue ist
				rescue();
			} else {
				fahre(0, 0, 0);
				led(0, 0, 1);
				Serial2.println(6);
			}
		} else {
			// Linienverfolgung
			int x = readString.toInt();

			if (x < 200 && x > -200) {

				int motorSpeedL = MOTORSPEED + x * MULTIPLIKATOR;
				int motorSpeedR = MOTORSPEED - x * MULTIPLIKATOR;

				if (getYOrientation() > 15.00) {   
					fahre(motorSpeedL * 1.5, motorSpeedR * 1.5, 0); 
				} else if (getYOrientation() < -15.00) {
					fahre(motorSpeedL * 0.5, motorSpeedR * 0.5, 0);
				} else {
					fahre(motorSpeedL, motorSpeedR, 0);
				}
			}
			doseFalsch();
		}
	}
}






void beep(int zeit) {
	digitalWrite(buzzerPin, HIGH);
	delay(zeit);
	digitalWrite(buzzerPin, LOW);
}
void fahre(int links, int rechts, int zeit) {

	if (rechts > 255) {
		rechts = 255;
	}
	if (rechts < -255) {
		rechts = -255;
	}
	if (links > 255) {
		links = 255;
	}
	if (links < -255) {
		links = -255;
	}


	if (rechts == 0) {
		digitalWrite(motorB1, LOW);
		digitalWrite(motorB2, LOW);
		analogWrite(motorBpwm, 0);
	}
	if (links == 0) {
		digitalWrite(motorA1, LOW);
		digitalWrite(motorA2, LOW);
		analogWrite(motorApwm, 0);
	}

	if (rechts > 0) {
		digitalWrite(motorB1, LOW);
		digitalWrite(motorB2, HIGH);
		analogWrite(motorBpwm, rechts);
	}
	if (rechts < 0) {
		digitalWrite(motorB1, HIGH);
		digitalWrite(motorB2, LOW);
		analogWrite(motorBpwm, -rechts);
	}
	if (links > 0) {
		digitalWrite(motorA1, LOW);
		digitalWrite(motorA2, HIGH);
		analogWrite(motorApwm, links);
	}
	if (links < 0) {
		digitalWrite(motorA1, HIGH);
		digitalWrite(motorA2, LOW);
		analogWrite(motorApwm, -links);
	}
	delay(zeit);
}
void dreheZu(float pwunschpos) {	
	if (pwunschpos > getXOrientation()) {
		while (getXOrientation() < pwunschpos - 2.0) {
			fahre(130, -130, 0);
		}
	} else {
		while (getXOrientation() > pwunschpos + 2.0) {
			fahre(-130, 130, 0);
		}
	}
	fahre(0, 0, 0);
}

void drehe(float deg) {
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
				fahre(130, -130, 0);
			}
		} else {
			if (endPos < 359.9999) {
				//pln("drehe einfach bis Wunschpos2");
				while (getXOrientation() < endPos) {
					fahre(130, -130, 0);
				}
			} else {
				pln("drehe bis 0 und dann bis zur Wunschpos");
				while (getXOrientation() > 1.0) { //drehe bis 0
					fahre(130, -130, 0);
				}
				endPos = endPos - 360.0;
				while (getXOrientation() < endPos) {
					fahre(130, -130, 0);
				}
			}
		}
		fahre(-130, 130, 40);
	} else {
		if (startPos >= 0 && startPos < 182) {
			if (endPos >= 0.0) {
				//pln("drehe einfach bis Wunschposition (linksherum)");
				while (getXOrientation() > endPos) {
					fahre(-130, 130, 0);
				}
			} else {
				//pln("drehe bis 0 und dann bis Wunschposition (linksherum)");
				while (getXOrientation() < 359.0) {
					fahre(-130, 130, 0);
				}
				endPos = endPos + 360;
				while (getXOrientation() > endPos) {
					fahre(-130, 130, 0);
				}
			}
		} else {
			//pln("drehe einfach bis Wunschposition (linksherum)");
			while (getXOrientation() > endPos) {
				fahre(-130, 130, 0);
			}
		}
		fahre(130, -130, 40);
	}
	fahre(0, 0, 0);
}

void p(String txt) {
	Serial.print(txt);
}
void pln(String txt) {
	Serial.println(txt);
}
void batteryCheck() {
	int val = analogRead(voltageDividor);
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
		fahre(0, 0, 0);
		led(0, 0, 1);
		beep(1000000);
	}
}
void greiferRunter() {
	servoSeil.attach(22);
	servoSeil.write(180); //Seil locker machen, um Kugel aufzunehmen
	delay(700);
	servoSeil.detach();


	servoGreifer.attach(23);
	servoGreifer.write(180); //Arm runter
	delay(900);
	servoGreifer.detach();
}

void greiferHalbRunter() {
	servoSeil.attach(22);
	servoSeil.write(180); //Seil locker machen, um Kugel aufzunehmen
	delay(700);
	servoSeil.detach();


	servoGreifer.attach(23);
	servoGreifer.write(140); //Arm runter
	delay(900);
	servoGreifer.detach();
}

void greiferHoch() {
	servoSeil.attach(22);
	servoSeil.write(0); //Seil fest machen, damit er Kugel nicht verliert
	delay(700);
	servoSeil.detach();


	servoGreifer.attach(23);
	servoGreifer.write(30); //Arm hoch
	delay(900);
	servoGreifer.detach();
}

int distance() {
	//returns distance of the front laser sensor

	int distance = laserVorne.readRangeContinuousMillimeters();
	//Serial.println(distance);
	if (laserVorne.timeoutOccurred()) {
		Serial.print("TIMEOUT -> check connections");
	}

	return distance;
}

int distanceAvg() {
	int distanceValues[] = {distance(), distance(), distance(), distance(), distance()}; //erstellt array mit 5 Sensorwerten

	int min = distanceValues[0];
	int max = distanceValues[0];

	if (distanceValues[1] < min) {
		min = distanceValues[1];
	}

	if (distanceValues[2] < min) {
		min = distanceValues[2];
	}

	if (distanceValues[3] < min) {
		min = distanceValues[3];
	}

	if (distanceValues[4] < min) {
		min = distanceValues[4];
	}




	if (distanceValues[1] > max) {
		max = distanceValues[1];
	}

	if (distanceValues[2] > max) {
		max = distanceValues[2];
	}

	if (distanceValues[3] > max) {
		max = distanceValues[3];
	}

	if (distanceValues[4] > max) {
		max = distanceValues[4];
	}

	int avg = (((distanceValues[0] + distanceValues[1] + distanceValues[2] + distanceValues[3] + distanceValues[4]) - min) - max) / 3; //bildet Durchschnitt der mittleren 3 Messungen
	return avg;
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
void doseFalsch() {
	if (distance() < 50) {
		if (distance() < 60) { //checkt, ob wirklich ein Hindernis erkannt wurde
			if (distance() < 60) {
				fahre(0, 0, 0);
				led(1, 1, 1);
				fahre(-255, -255, 200);
				drehe(50);
				fahre(255, 255, 200);
				for (int i = 0; i < 10; i++) {
					fahre(255, 255, 110);
					drehe(-5);
				}
				fahre(0, 0, 500);
				fahre(255, 255, 300);
				drehe(38);
				fahre(-255, -255, 20);
			}
		}
	}
}
void dose() {
	if (distance() < 50) {
		if (distance() < 60) { //checkt, ob wirklich ein Hindernis erkannt wurde
			if (distance() < 60) {
				fahre(0, 0, 0);
				led(1, 1, 1);
				//Serial2.print("Dose");
				beep(50);
				fahre(-150, -150, 200);
				fahre(255, -255, neunzigGrad / 1.3);
				fahre(255, 255, 100);
				int x = 200;
				Serial2.flush();
				beep(50);
				fahre(0, 0, 0);
				delay(1000);
				led(0, 0, 0);
				/*while (Serial2.available()) { //hängt sich in dieser loop auf Fehlercode: jdhjkdfhg
					delay(4);
					Serial.read();
					digitalWrite(ledrotPin, HIGH);
					}*/
				digitalWrite(ledrotPin, LOW);

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
					fahre(30, 255, 0);
					a = -a;
					delay(20);
				}
				led(0, 1, 0);
				fahre(255, 255, 300);
				fahre(255, -255, 500);
				fahre(255, 255, 100);
				fahre(0, 0, 0);
				beep(50);
			}
			led(0, 0, 0);
		}
	}
}

void ausrichten() {
	bool ungerade = true;
	int last = distanceAvg();
	fahre(150, -150, 200);
	fahre(0, 0, 0);
	int current = distanceAvg();

	if (current < last) {
		while (ungerade) {
			led(1, 1, 1);
			last = distanceAvg();
			fahre(150, -150, 50);
			fahre(0, 0, 0);
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
				fahre(0, 0, 0);
				led(0, 0, 0);
				fahre(-255, -255, 3000);
				fahre(0, 0, 0);
				delay(100000);
				//ungerade = false;
				//break;
			}
		}
	}
}

void rescue() {
	fahre(0, 0, 0);
	beep(20);
	boolean rescue = true;
	while (rescue) {
		if (Serial2.available() > 0) {
			String incomingString = Serial2.readString();
			//incomingString.trim();
			String xval = getValue(incomingString, ':', 0);
			String yval = getValue(incomingString, ':', 1);
			String zval = getValue(incomingString, ':', 2);
			int motorLinks = xval.toInt();
			int motorRechts = yval.toInt();
			int zeit = zval.toInt();   
			pln(incomingString);
			if (incomingString == "greiferHoch") {
				fahre(0, 0, 0);
				greiferHoch();
				Serial2.println(1);
			} else if (incomingString == "greiferRunter") {
				fahre(0, 0, 0);
				greiferRunter();
				Serial2.println(1);
			} else if (incomingString == "setzeUrsprung") {
				ursprung = getXOrientation();
				beep(50);
				p("Ursprung gesetzt auf Grad: ");
				p(ursprung);
				pln("");
				Serial2.println(1);
			} else if (incomingString == "dreheZuUrsprung") {
				dreheZu(ursprung);
				Serial2.println(1);
			} else if (incomingString == "exit") {
				rescueFlag == true;
				fahre(0, 0, 0);
				Serial2.println(1);
				fahre(0, 0, 3000);
				return;
			} else if (incomingString == "fahreZuEckeUndLadeKugelAb") {
				drehe(90);
				while (distanceAvg() > 100) {
					fahre(255, 255, 0);
					led(0, 0, 1);
				}
				fahre(255, 255, 700);
				fahre(-255, -255, 500);
				drehe(90);
				while (distanceAvg() > 130) {
					fahre(255, 255, 0);
					led(0, 0, 1);
				}
				fahre(0, 0, 0);
				drehe(90);
				while (distanceAvg() > 400) {
					fahre(255, 255, 0);
				}
				fahre(0, 0, 0);
				drehe(45);
				fahre(255, 255, 650);
				drehe(90);
				fahre(-255, -255, 500);
				fahre(0, 0, 500);
				greiferHalbRunter();
				greiferHoch();
				fahre(255, 255, 1500);
				dreheZu(ursprung);
				fahre(255, 255, 800);
				drehe(90);
				while (distanceAvg() > 100) {
					fahre(255, 255, 0);
					led(0, 0, 1);
				}
				fahre(255, 255, 700);
				fahre(-255, -255, 300);
				drehe(-90);
				fahre(0, 0, 0);
				Serial2.println(1);
			} else {
				/*
				p(motorLinks);
				p("  ");
				p(motorRechts);
				pln("");
				*/
				if (motorLinks == 0 && motorRechts == 0) {
					if (zeit == 0) { //drehe dich zum Ursprung

					} else {
						drehe(zeit);
						Serial2.println(1);
					}
				} else {
					fahre(motorLinks, motorRechts, zeit);
					fahre(0, 0, 0);
					Serial2.println(1);
				}
			}
		}
	}
}

String getValue(String data, char separator, int index) { //returns ints from mutliple Strings seperated by : 
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
	if (red > 1) {
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
	}
	digitalWrite(ledgruenPin, green);
	digitalWrite(ledgelbPin, yellow);
	digitalWrite(ledrotPin, red);
}
