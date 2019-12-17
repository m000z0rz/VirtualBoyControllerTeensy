#include <Arduino.h>

#define LATCH_PIN 0
#define CLOCK_PIN 1
#define DATA_PIN 2

#define LATCH_LOW digitalWrite(LATCH_PIN, LOW)
#define LATCH_HIGH digitalWrite(LATCH_PIN, HIGH)
#define CLOCK_LOW digitalWrite(CLOCK_PIN, LOW)
#define CLOCK_HIGH digitalWrite(CLOCK_PIN, HIGH)
#define DATA_READ digitalRead(DATA_PIN)

#define PRESSED(i, mask) (rawData[i] & mask && !(lastData[i] & mask)
#define RELEASED(i, mask) (!(rawData[i] & mask) && !(lastData))

// A lot of this is taken directly from https://github.com/raphnet/virtualboy_usb/blob/master/virtualboy.c

void sendKeyboardUpdate(uint8_t byteI, uint8_t mask, uint16_t key);

// bool vlup = false;
// bool vldown = false;
// bool vlleft = false;
// bool vlright = false;

// bool vrup = false;
// bool vrdown = false;
// bool vrleft = false;
// bool vrright = false;

// bool vselect = false;
// bool vstart = false;

// bool va = false;
// bool vb = false;

// bool vl = false;
// bool vr = false;

unsigned pressed[2];
unsigned lastPressed[2];
unsigned changed[2];

void setup() {
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT_PULLUP);

  LATCH_LOW;
  CLOCK_HIGH;

  delayMicroseconds(10);

  Serial.begin(9600);
}



void loop() {

 	int i,j;
	unsigned char tmp=0;
  pressed[0] = 0;
  pressed[1] = 0;
	//int x=128,y=128,rx=128,ry=128;
	//char left_dpad_analog = 0;

	LATCH_HIGH;
	delayMicroseconds(12);
	LATCH_LOW;

	for (j=0; j<2; j++)
	{
		for (i=0; i<8; i++)
		{
			delayMicroseconds(6);
			CLOCK_LOW;

			tmp <<= 1;
			if (DATA_READ) { tmp |= 1; }

      delayMicroseconds(6);
			CLOCK_HIGH;
		}

		pressed[j] = tmp;
	}

  Serial.print("pressed: ");
  Serial.print(pressed[0], HEX);
  Serial.print(pressed[1], HEX);

  // bits:
  //  0x0001 lright
  //  0x0002 lleft
  //  0x0004 ldown
  //  0x0008 lup
  //  0x0010 start
  //  0x0020 select
  //  0x0040 rleft
  //  0x0080 rdown
  //  0x0100
  //  0x0200
  //  0x0400 rup
  //  0x0800 rdown
  //  0x1000 r
  //  0x2000 l
  //  0x4000 a
  //  0x8000 b

  // Left left
  changed[0] = pressed[0] ^ lastPressed[0];
  changed[1] = pressed[1] ^ lastPressed[1];

  Serial.print(" changed: ");
  Serial.print(changed[0], HEX);
  Serial.print(changed[1], HEX);
  Serial.println();

  // sendKeyboardUpdate(0, 0x08, KEY_W); // lup
  // sendKeyboardUpdate(0, 0x04, KEY_S); // ldown
  // sendKeyboardUpdate(0, 0x02, KEY_A); // lleft
  // sendKeyboardUpdate(0, 0x01, KEY_D); // lright

  // sendKeyboardUpdate(1, 0x40, KEY_M); // a
  // sendKeyboardUpdate(1, 0x80, KEY_N); // b
  // sendKeyboardUpdate(0, 0x10, KEY_ENTER); // start
  // sendKeyboardUpdate(0, 0x20, KEY_RIGHT_SHIFT); // select
  // sendKeyboardUpdate(1, 0x20, KEY_Q); // l
  // sendKeyboardUpdate(1, 0x10, KEY_O); // r

  // sendKeyboardUpdate(1, 0x40, KEY_I); // rup
  // sendKeyboardUpdate(0, 0x80, KEY_K); // rdown
  // sendKeyboardUpdate(0, 0x40, KEY_J); // rleft
  // sendKeyboardUpdate(1, 0x80, KEY_L); // rright

  lastPressed[0] = pressed[0];
  lastPressed[1] = pressed[1];

  // put your main code here, to run repeatedly:
  // Joystick.X(1023);
  // delay(500);
  // Joystick.button(1, true);
  // delay(500);
  // Joystick.button(1, false);
  // delay(500);
  // Joystick.X(0);
  // delay(500);

}

void sendKeyboardUpdate(uint8_t byteI, uint8_t mask, uint16_t key)
{
  if (changed[byteI] & mask)
  {
    if (pressed[byteI] & mask) { Keyboard.press(key); }
    else { Keyboard.release(key); }
  }
}