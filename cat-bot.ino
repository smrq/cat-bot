#include <Arduino.h>
#include <EEPROM.h>
#include <ServoEx.h>
#include "serial_printf.h"

#define PIN_BT_RX 9
#define PIN_BT_TX 10
#define PIN_SERVO_X 14
#define PIN_SERVO_Y 15
#define PIN_LASER 20

#define CENTER_X 81
#define CENTER_Y 45

#define SERVO_SPEED 300
#define DELAY_STEP 50
#define DELAY_MULTIPLIER 5

#define BLUETOOTH Serial2

enum {
	MODE_STANDBY = 0,
	MODE_AUTOPLAY = 1,
	MODE_CALIBRATE = 2
};

enum {
	MSG_DEBUG = '?',
	MSG_SAVE_SETTINGS = 'S',
	MSG_AUTOPLAY = 'A',
	MSG_AUTO_SPEED_UP = '+',
	MSG_AUTO_SPEED_DOWN = '-',
	MSG_CALIBRATE = 'C',
	MSG_CALIBRATE_UP = 'u',
	MSG_CALIBRATE_DOWN = 'd',
	MSG_CALIBRATE_LEFT = 'l',
	MSG_CALIBRATE_RIGHT = 'r'
};

struct Point {
	uint8_t x;
	uint8_t y;
};

struct Settings {
	int version;
	int delayTime;
	struct Point corners[4];
};

Settings settings;
int mode;
unsigned long nextAutoMove;
int calibrationX;
int calibrationY;
int calibrationIndex;

ServoEx servoX;
ServoEx servoY;

float randomFloat() {
	return (float)rand() / (float)RAND_MAX;
}

float lerp(float t, float p0, float p1) {
	return p0 + t * (p1 - p0);
}

void interpolateQuad(float u, float v, struct Point corners[4], int &x, int &y) {
	float x1 = lerp(u, (float)corners[0].x, (float)corners[1].x);
	float x2 = lerp(u, (float)corners[3].x, (float)corners[2].x);
	float y1 = lerp(u, (float)corners[0].y, (float)corners[1].y);
	float y2 = lerp(u, (float)corners[3].y, (float)corners[2].y);

	x = (int)lerp(v, x1, x2);
	y = (int)lerp(v, y1, y2);
}

void moveToPosition(int x, int y, int speed) {
	serial_printf(BLUETOOTH, "moving to position [%d,%d]...\r\n", x, y);
	ServoGroupMove.start();
	servoX.write(x);
	servoY.write(y);
	ServoGroupMove.commit(speed);
	ServoGroupMove.wait(-1);
	serial_printf(BLUETOOTH, "moved.\r\n");
}

void printSettings(Stream &stream) {
	serial_printf(stream, "  delayTime: %d\r\n", settings.delayTime);
	serial_printf(stream, "  corners:\r\n");
	for (int i = 0; i < 4; ++i) {
		serial_printf(stream, "    %d: [%d,%d]\r\n", i, settings.corners[i].x, settings.corners[i].y);
	}
}

void setup() {
	Serial.begin(9600);
	serial_printf(Serial, "cat-bot starting...\r\n");

	pinMode(PIN_BT_RX, INPUT);
	pinMode(PIN_BT_TX, OUTPUT);
	BLUETOOTH.begin(9600);

	serial_printf(Serial, "bluetooth initialized.\r\nfurther logs on bluetooth device.\r\n");
	serial_printf(BLUETOOTH, "bluetooth initialized.\r\n");

	pinMode(PIN_LASER, OUTPUT);
	digitalWrite(PIN_LASER, 0);
	serial_printf(BLUETOOTH, "laser initialized.\r\n");

	pinMode(PIN_SERVO_X, OUTPUT);
	pinMode(PIN_SERVO_Y, OUTPUT);
	servoX.attach(PIN_SERVO_X);
	servoY.attach(PIN_SERVO_Y);
	servoX.write(CENTER_X);
	servoY.write(CENTER_Y);
	serial_printf(BLUETOOTH, "servos initialized.\r\n");

	EEPROM.get(0, settings);
	if (settings.version != 1) {
		settings.version = 1;
		settings.delayTime = 300;
		settings.corners[0].x = CENTER_X - 5;
		settings.corners[0].y = CENTER_Y - 5;
		settings.corners[1].x = CENTER_X + 5;
		settings.corners[1].y = CENTER_Y - 5;
		settings.corners[2].x = CENTER_X + 5;
		settings.corners[2].y = CENTER_Y + 5;
		settings.corners[3].x = CENTER_X - 5;
		settings.corners[3].y = CENTER_Y + 5;
		serial_printf(BLUETOOTH, "settings initialized:\r\n");
		printSettings(BLUETOOTH);
	} else {
		serial_printf(BLUETOOTH, "settings loaded:\r\n");
		printSettings(BLUETOOTH);
	}

	mode = MODE_STANDBY;

	serial_printf(BLUETOOTH, "cat-bot online.\r\n");
}

void flashLaser(bool stayOn) {
	for (int i = 0; i < 5; ++i) {
		digitalWrite(PIN_LASER, 1);
		delay(100);
		digitalWrite(PIN_LASER, 0);
		delay(100);
	}
	if (stayOn) {
		digitalWrite(PIN_LASER, 1);
	}
}

void handleGlobalCmd(char cmd) {
	switch (cmd) {
		case MSG_DEBUG:
			serial_printf(Serial, "Debug message!\r\n");
			serial_printf(Serial,
				"mode: %d\r\nnextAutoMove: %d\r\ncalibrationXY: %d,%d\r\ncalibrationIndex: %d\r\n",
				mode, nextAutoMove, calibrationX, calibrationY, calibrationIndex);
			printSettings(Serial);

			serial_printf(BLUETOOTH, "Debug message!\r\n");
			serial_printf(BLUETOOTH,
				"mode: %d\r\nnextAutoMove: %d\r\ncalibrationXY: %d,%d\r\ncalibrationIndex: %d\r\n",
				mode, nextAutoMove, calibrationX, calibrationY, calibrationIndex);
			printSettings(BLUETOOTH);
			break;

		case MSG_SAVE_SETTINGS:
			EEPROM.put(0, settings);
			flashLaser(0);
			serial_printf(BLUETOOTH, "settings saved:");
			printSettings(BLUETOOTH);
			break;
	}
}

void handleStandbyCmd(char cmd) {
	switch (cmd) {
		case MSG_AUTOPLAY:
			mode = MODE_AUTOPLAY;
			nextAutoMove = millis();
			digitalWrite(PIN_LASER, 1);
			serial_printf(BLUETOOTH, "autoplay mode.\r\n");
			break;

		case MSG_CALIBRATE:
			mode = MODE_CALIBRATE;
			calibrationX = settings.corners[0].x;
			calibrationY = settings.corners[0].y;
			calibrationIndex = 0;
			serial_printf(BLUETOOTH, "calibration mode.\r\n");
			digitalWrite(PIN_LASER, 1);
			for (int i = 0; i < 4; ++i) {
				moveToPosition(settings.corners[i].x, settings.corners[i].y, SERVO_SPEED);
			}
			moveToPosition(calibrationX, calibrationY, SERVO_SPEED);
			break;
	}
}

void handleAutoplayCmd(char cmd) {
	switch (cmd) {
		case MSG_AUTOPLAY:
			mode = MODE_STANDBY;
			digitalWrite(PIN_LASER, 0);
			moveToPosition(CENTER_X, CENTER_Y, SERVO_SPEED);
			serial_printf(BLUETOOTH, "standby mode.\r\n");
			break;

		case MSG_AUTO_SPEED_UP:
			if (settings.delayTime >= DELAY_STEP) {
				settings.delayTime -= DELAY_STEP;
				serial_printf(BLUETOOTH, "speed up | delay time is %d ms.\r\n", settings.delayTime);
			} else {
				serial_printf(BLUETOOTH, "speed is at maximum | delay time is %d ms.\r\n", settings.delayTime);
			}
			break;

		case MSG_AUTO_SPEED_DOWN:
			settings.delayTime += DELAY_STEP;
			serial_printf(BLUETOOTH, "speed down | delay time is %d ms.\r\n", settings.delayTime);
			break;
	}
}

void handleCalibrateCmd(char cmd) {
	switch (cmd) {
		case MSG_CALIBRATE_LEFT:
			calibrationX += 1;
			moveToPosition(calibrationX, calibrationY, 0);
			break;

		case MSG_CALIBRATE_RIGHT:
			calibrationX -= 1;
			moveToPosition(calibrationX, calibrationY, 0);
			break;

		case MSG_CALIBRATE_UP:
			calibrationY += 1;
			moveToPosition(calibrationX, calibrationY, 0);
			break;

		case MSG_CALIBRATE_DOWN:
			calibrationY -= 1;
			moveToPosition(calibrationX, calibrationY, 0);
			break;

		case MSG_CALIBRATE:
			settings.corners[calibrationIndex].x = calibrationX;
			settings.corners[calibrationIndex].y = calibrationY;
			serial_printf(BLUETOOTH, "corner %d set to [%d,%d].\r\n", calibrationIndex, calibrationX, calibrationY);
			flashLaser(1);

			if (++calibrationIndex < 4) {
				calibrationX = settings.corners[calibrationIndex].x;
				calibrationY = settings.corners[calibrationIndex].y;
				moveToPosition(calibrationX, calibrationY, SERVO_SPEED);
			} else {
				for (int i = 0; i < 4; ++i) {
					moveToPosition(settings.corners[i].x, settings.corners[i].y, SERVO_SPEED);
				}
				digitalWrite(PIN_LASER, 0);
				moveToPosition(CENTER_X, CENTER_Y, SERVO_SPEED);
				mode = MODE_STANDBY;
				serial_printf(BLUETOOTH, "calibration complete.\r\nstandby mode.\r\n");
			}
			break;
	}
}

void loop() {
	char cmd = 0;
	if (BLUETOOTH.available()) {
		cmd = BLUETOOTH.read();
		serial_printf(BLUETOOTH, "command received: %c\r\n", cmd);
	}

	handleGlobalCmd(cmd);

	switch (mode) {
		case MODE_STANDBY:
			handleStandbyCmd(cmd);
			break;

		case MODE_AUTOPLAY: {
			unsigned long t = millis();
			if (t >= nextAutoMove) {
				float u = randomFloat();
				float v = randomFloat();
				int x;
				int y;
				interpolateQuad(u, v, settings.corners, x, y);
				moveToPosition(x, y, SERVO_SPEED);
				nextAutoMove = t + random(settings.delayTime, settings.delayTime * DELAY_MULTIPLIER);
			}
			handleAutoplayCmd(cmd);
			break;
		}

		case MODE_CALIBRATE:
			handleCalibrateCmd(cmd);
			break;
	}
}
