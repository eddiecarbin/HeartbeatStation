
volatile float x = 0.0;
volatile float x1 = 0.0;
volatile float x2 = 0.0;

volatile float y = 0.0;
volatile float y1 = 0.0;
volatile float y2 = 0.0;
volatile float y3 = 0.0;

volatile float z = 0.0;
volatile float z1 = 0.0;
volatile float z2 = 0.0;
volatile float z3 = 0.0;

int ii = 0;
int k = 0;
int l;

void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
  delay(3000);

  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
}

void loop() {
  // put your main code here, to run repeatedly:
 // -------------------------------------------- Acquiring Data ------------------------------------------

  x = analogRead(A0);

  // ----------------------------------------------- Filtering --------------------------------------------

  // Filtering around 40 and 60 Hz (notch).
  y = 0.2012 * x - 0.3256 * x1 + 0.2012 * x2 + 0.3256 * y1 + 0.5975 * y2;

  // Low-pass Filter 100 Hz.
  z = 0.0495 * y + 0.1486 * y1 + 0.1486 * y2 + 0.0495 * y3 + 1.1619 * z1 - 0.6959 * z2 + 0.1378 * z3;

  x2 = x1;
  x1 = x;

  y3 = y2;
  y2 = y1;
  y1 = y;

  z3 = z2;
  z2 = z1;
  z1 = z;

  // Saving z value
  //timeVecto2[k] = z;

  k = (k + 1) % 512;
  Serial.println(z);
  delay(1);
}
