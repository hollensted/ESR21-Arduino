
// Dekodierung des Manchester-Codes // decoding the manchester code

// Pulsweite bei 488hz: 1000ms/488 = 2,048ms = 2048µs
// 2048µs / 2 = 1024µs (2 Pulse für ein Bit)
// pulse width at 488hz: 1000ms/488 = 2,048ms = 2048µs
// 2048µs / 2 = 1024µs (2 pulses for one bit)
const unsigned long pulse_width = 1024; // µs
// % Toleranz für Abweichungen bei der Pulsweite
const int percentage_variance = 10; // % tolerance for variances at the pulse width
// 1001 oder 0110 sind zwei aufeinanderfolgende Pulse ohne Übergang
// 1001 or 0110 are two sequential pulses without transition
const unsigned long double_pulse_width = pulse_width * 2;
// Berechnung der Toleranzgrenzen für Abweichungen
// calculating the tolerance limits for variances
const unsigned long low_width = pulse_width - (pulse_width *  percentage_variance / 100);
const unsigned long high_width = pulse_width + (pulse_width * percentage_variance / 100);
const unsigned long double_low_width = double_pulse_width - (pulse_width * percentage_variance / 100);
const unsigned long double_high_width = double_pulse_width + (pulse_width * percentage_variance / 100);
boolean volatile got_first = 0; // erster oder zweiter Puls für ein Bit? // first or second pulse for one bit?
volatile uint32_t last_bit_change = 0; // Merken des letzten Übergangs // remember the last transition
int volatile pulse_count; // Anzahl der empfangenen Pulse // number of received pulses
#define BIT_COUNT (pulse_count / 2)
byte volatile receiving; // Übertragungs-Flag // currently receiving?
const byte relay = D3;
const byte interruptPin = D1;
const int bit_number = 512; // Dataframe(ESR21) is 248 bits
byte volatile data_bits[bit_number / 8 + 1];
int volatile start_bit; // erstes Bit des Datenrahmens // first bit of data frame


boolean volatile data = false;
String SensorValue[17];
boolean first = true;
int calibrate[16];
long timer;
long interval = 60000;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //pinMode(relay, OUTPUT);
  delay(1000);
  start();
  //Serial.println(String(digitalPinToInterrupt(interruptPin)));
  timer = millis() + 5000;
  Serial.println("\n");
}

void loop() {
  if (!data && !receiving && millis() >= timer)start();
  if (data)
  {
    //first = true; // Override compare
    prepare();
    //printHex();
    //printFrame();
    if (check_device() == 1)
    {
      int out = data_bits[8];
      double T3 = convTemperature(3);
      //if (T3 < 35 && out == 0) if (digitalRead(relay) != HIGH) digitalWrite(relay, HIGH);
      //if (T3 > 40 || out == 1) if (digitalRead(relay) == HIGH) digitalWrite(relay, LOW);
      //digitalWrite(relay, HIGH);
      timer = timer + interval;
      Serial.println("T1: " + String(convTemperature(1)) + " T2: " + String(convTemperature(2)) + " T3: " + String(convTemperature(3)) + " Out: " + String(data_bits[8]) + " A1: " + String(data_bits[9]) + " Relay: " + String(digitalRead(relay))); //+" Power: "+String(kw()));//+" kWh: "+String(kwh())+" mWh: "+String(mwh()));
    }
    else start();
    data = false;
  }
}

void start() {
  pulse_count = got_first = last_bit_change = 0;
  receiving = true;

  // bei einem CHANGE am Daten-Pin wird pin_changed aufgerufen
  // on a CHANGE on the data pin pin_changed is called
  attachInterrupt(digitalPinToInterrupt(interruptPin), pin_changed, CHANGE);
}

void stop2() {
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  receiving = false;
  data = true;
  //Serial.println("Frame! "+String(BIT_COUNT));
}

ICACHE_RAM_ATTR void pin_changed() {
  byte val = digitalRead(interruptPin); // Zustand einlesen // read state
  uint32_t time_diff = micros() - last_bit_change;
  last_bit_change = micros();
  //Serial.print(String(time_diff)+" ");
  //if (time_diff > 2000)Serial.print(String(time_diff)+" ");
  // einfache Pulsweite? // singe pulse width?
  if (time_diff >= low_width && time_diff <= high_width) {
    process_bit(val);
    return;
  }
  // doppelte Pulsweite? // double pulse width?
  if (time_diff >= double_low_width && time_diff <= double_high_width) {
    process_bit(!val);
    process_bit(val);
    return;
  }
}

void process_bit(byte b) {
  // den ersten Impuls ignorieren // ignore first pulse
  pulse_count++;
  //if (pulse_count < 2)return;

  if (b)data_bits[BIT_COUNT / 8] |= 1 << BIT_COUNT % 8; // Bit setzen // set bit
  else data_bits[BIT_COUNT / 8] &= ~(1 << BIT_COUNT % 8); // Bit löschen // clear bit

  if (BIT_COUNT == bit_number)//Process::bit_number)
    stop2(); // stop receiving when data frame is complete

}
boolean prepare() {
  start_bit = analyze(); // Anfang des Datenrahmens finden // find the data frame's beginning
  // invertiertes Signal? // inverted signal?
  if (start_bit == -1) {
    invert(); // erneut invertieren // invert again
    start_bit = analyze();
  }
  trimBits(); // Start- und Stopbits entfernen // remove start and stop bits
  return check_device(); // nur für die UVR1611
}

int analyze() {
  byte sync;
  // finde SYNC (16 * aufeinanderfolgend 1) // find SYNC (16 * sequential 1)
  for (int i = 0; i < bit_number; i++) {
    if (read_bity(i))
      sync++;
    else
      sync = 0;
    if (sync == 16) {
      // finde erste 0 // find first 0
      while (read_bity(i) == 1)
        i++;
      return i ; // Anfang des Datenrahmens // beginning of data frame
    }
  }
  // kein Datenrahmen vorhanden. Signal überprüfen?
  return -1; // no data frame available. check signal?
}

void invert() {
  //Serial.println("Invert!");
  for (int i = 0; i < bit_number; i++)
    write_bity(i, read_bity(i) ? 0 : 1); // jedes Bit umkehren // invert every bit
}

byte read_bity(int pos) {
  int row = pos / 8; // Position in Bitmap ermitteln // detect position in bitmap
  int col = pos % 8;
  return (((data_bits[row]) >> (col)) & 0x01); // Bit zurückgeben // return bit
}

void write_bity(int pos, byte set) {
  int row = pos / 8; // Position in Bitmap ermitteln // detect position in bitmap
  int col = pos % 8;
  if (set)
    data_bits[row] |= 1 << col; // Bit setzen // set bit
  else
    data_bits[row] &= ~(1 << col); // Bit löschen // clear bit
}

void trimBits() {
  for (int i = start_bit, bity = 0; i < bit_number; i++) {
    int offset = i - start_bit;
    // Start- und Stop-Bits ignorieren:
    // Startbits: 0 10 20 30, also  x    % 10 == 0
    // Stopbits:  9 19 29 39, also (x+1) % 10 == 0
    // ignore start and stop bits:
    // start bits: 0 10 20 30, also  x    % 10 == 0
    // stop bits:  9 19 29 39, also (x+1) % 10 == 0
    if (offset % 10 && (offset + 1) % 10) {
      write_bity(bity, read_bity(i));
      bity++;
    }
  }
}
void trim3()
{
  int bity = 0;
  for (int i = start_bit - 16; i < bit_number; i++) {
    write_bity(bity, read_bity(i));
    write_bity(i, 0);
    bity++;
  }
}
void shift(int shifty)
{
  int bity = bit_number + shifty;
  for (int i = bit_number; i > 0; i--) {
    write_bity(bity, read_bity(i));
    bity--;
  }
}

boolean check_device() {
  // Datenrahmen von einer ESR21? // data frame of ESR21?
  byte r = 0;
  for (int a = 0; a < 21; a++)r = r + data_bits[a];
  if (r == data_bits[21]) // Checksum match?
    if (data_bits[0] == 0x70 && data_bits[1] == 0x8f)// 0x70 -> Gerätekennung// 0x8f is 0x70 inverted
      return true;
    else
      return false;
}

void printFrame()
{
  for (int i = 0; i < 248; i++) {
    Serial.print(String(read_bity(i)));
  }
  Serial.println();
}
void printHex()
{
  for (int a = 0; a < 22; a++) {
    if (data_bits[a] < 16)Serial.print("0");
    Serial.print(String(data_bits[a], HEX));
    if (!((a + 1) % 48))Serial.println();
  }
  Serial.println();
}
void printBits (int x) // Print bits (backwards) 76543210
{
  for (int i = x * 8 + 7; i >= x * 8; i--) {
    Serial.print(String(read_bity(i)));
  }
  Serial.println();
}
double convTemperature(int x) // Temp = 1/10 * ( Low-Byte + 256*High-Byte – 65536 ) AND remove bit 6,5&4 from high byte
{
  byte sensor_low = data_bits[x * 2];
  byte sensor_high = data_bits[x * 2 + 1 ];
  sensor_high = sensor_high & B10001111;  // x010xxxx = temperature
  if ( sensor_high & B10000000 )sensor_high = sensor_high | B11110000; // Test if negative
  int number = sensor_high << 8 | sensor_low;
  return (double)number / 10;
}
double kw ()
{
  double result = (double)(data_bits[24] + data_bits[25] * 256) / 10;
  if (result >= 0)
    return result;
  else return 0;
}
double kwh ()
{
  return (double)(data_bits[26] + data_bits[27] * 256) / 10;
}
double mwh ()
{
  return data_bits[28] + data_bits[29] * 256;
}
int outPut()
{
  if (data_bits[21] & 128)return -1; //Check if output is active bit7 = 0
  else
    return data_bits[21] & B00011111;
}
