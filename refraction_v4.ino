/***************************************************
  This is the code for the low-cost Refraction Seismograph

  ----> https://github.com/timdeangeo/RefractionSeismograph

  Written by Tim Dean, Curtin University 2018.
  GNU General Public Licence license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>             // Display
#include <Adafruit_ILI9341.h>
#include <Wire.h>            // FT6206
#include <Adafruit_FT6206.h>
#include <SparkFunLSM9DS1.h> // Sensor stick 
#include <SD.h>              // SD card
#include <Time.h>

int software_version = 3;

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();

// The display also uses hardware SPI, plus #9 & #10
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, 22);

// Size of the color selection boxes and the paintbrush size

#define BOXSIZE 40
#define PENRADIUS 3

// Size is 240 x 320
#define FRAME_X 0
#define FRAME_Y 0
#define FRAME_W 240
#define FRAME_H 100

// Record button
#define GREENBUTTON_X1 0
#define GREENBUTTON_Y1 0
#define GREENBUTTON_X2 FRAME_W
#define GREENBUTTON_Y2 100

// Position buttons
#define POSITION_X1 0
#define POSITION_X2 70
#define POSITION_Y1 105
#define POSITION_Y2 170
#define POSITION_GAP 5
#define FIRST_BUTTON_DIFF 20

// Settings button
#define SETTINGSBUTTON_X1 0
#define SETTINGSBUTTON_X2 100
#define SETTINGSBUTTON_Y1 245
#define SETTINGSBUTTON_Y2 320

// Next button
#define NEXTBUTTON_X1 105
#define NEXTBUTTON_X2 240
#define NEXTBUTTON_Y1 245
#define NEXTBUTTON_Y2 320

int oldcolor, currentcolor;

// Accelerometer  section ------------------------------------------------
char gyroSample[120]; // Gyro sample character array
char gyro2Sample[120]; // Gyro sample character array
// If the length of this array is not big enough then you get corrupted values (but the first one is ok!).
LSM9DS1 imu;
LSM9DS1 imu2;

#define LSM9DS1_M    0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG   0x6B // Would be 0x6A if SDO_AG is LOW

#define LSM9DS1_M2   0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG2   0x6A // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // Gyro, temp, and acceleration measurements
int numberGyroMeasurements = 1000;

// SD Card section ------------------------------------------------
#define SD_CS 4
File dataFile;
bool sd_connected;

// Geometry section ------------------------------------------------
int position_1 = 1;
int position_2 = 2;
int increment = 1; // Position increment
int position_counter = 1;
int recording_time = 5; // Record length
int sample_rate = 400; // samples/sec (this is for y-axis only, if you add axes this will decrease)
int num_samples = sample_rate * recording_time; // Conversion from time to number of samples

// Mode 1 = recording, 2 = setup
int mode = 1;


void setup(void) {
  while (!Serial);     // used for leonardo debugging

  Serial.begin(9600);
  Serial.println(F("Cap Touch Paint!"));

  if (! ctp.begin(40)) {  // pass in 'sensitivity' coefficient
    Serial.println("Couldn't start FT6206 touchscreen controller");
    //while (1);
  }

  tft.begin();

  Serial.println("Capacitive touchscreen started");

  // ----------------------------------------------
  // SD Card
  sd_connected = 1;
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    sd_connected = 0;
  }

  Serial.println("OK!");

  tft.fillScreen(ILI9341_BLACK);
  int8_t i = -2;
  // The logo must be placed on the SD card for this to work
  bmpDraw("cu_logo.bmp", 0, 0);
  delay(3000);

  // ----------------------------------------------
  // Gyro/Accel setup parameters
  // IMU 1
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.accel.scale = 2;
  imu.settings.accel.sampleRate = 6; // 952 Hz
  imu.settings.accel.highResEnable = true;

  // IMU 2
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.mAddress = LSM9DS1_M2;
  imu2.settings.device.agAddress = LSM9DS1_AG2;
  imu2.settings.accel.scale = 2;
  imu2.settings.accel.sampleRate = 6; // 952 Hz
  imu2.settings.accel.highResEnable = true;

  // Start the IMUs
  Serial.println("   STARTING IMU 1...");
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 10);
  tft.print("REF. ACQ. V");
  tft.println(software_version);
  tft.setCursor(10, 40);
  if (sd_connected == 1) {
    tft.setTextColor(ILI9341_GREEN);
    tft.println("SD CARD CONNECTED");
  }
  else {
    tft.setTextColor(ILI9341_RED);
    tft.println("SD CARD NOT CON.");
  }

  tft.setCursor(10, 70);
  if (!imu.begin())  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("   IMU 1 NOT CONNECTED");
    tft.setTextColor(ILI9341_RED);
    tft.println("SEN. 1 NOT CON.");
  }
  else  {
    Serial.println("   IMU 1 CONNECTED");
    get_accel_sample();
    Serial.println(gyroSample);
    tft.setTextColor(ILI9341_GREEN);
    tft.println("SEN. 1 CONNECTED");
  }

  tft.setCursor(10, 100);
  Serial.println("   STARTING IMU 2...");
  if (!imu2.begin())  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("   IMU NOT 2 CONNECTED");
    tft.setTextColor(ILI9341_RED);
    tft.println("SEN. 2 NOT CON.");
  }
  else  {
    Serial.println("   IMU 2 CONNECTED");
    get_accel2_sample();
    Serial.println(gyro2Sample);
    tft.setTextColor(ILI9341_GREEN);
    tft.println("SEN. 2 CONNECTED");
  }
  delay(2000);
  setupRecordingScreen();
}

void loop() {
  // Wait for a touch
  if (! ctp.touched()) {
    return;
  }

  // Retrieve a point
  TS_Point p = ctp.getPoint();

  // flip it around to match the screen.
  p.x = map(p.x, 0, 240, 240, 0);
  p.y = map(p.y, 0, 320, 320, 0);

  if (p.y < GREENBUTTON_Y2)  {
    //liveStreamGyro();
    if (mode == 1) {
      logData();
    }
    else if (mode == 2) {
      listFiles();
    }
  }
  else if (p.y < POSITION_Y2)   {
    // Red position
    if (mode == 1) {
      if (p.x > POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
        if (p.x < POSITION_X2 + POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
          position_1 = position_1 - increment;
        }
        else  {
          position_1 = position_1 + increment;
        }
      }
      position1Btn();
    }
    else if (mode == 2) { // Increment recording time
      if (p.x > POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
        if (p.x < POSITION_X2 + POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
          recording_time = recording_time - 1;
        }
        else  {
          recording_time = recording_time + 1;
        }
      }
      num_samples = sample_rate * recording_time;
      recordingTimeBtn();
    }
  }
  else if (p.y <  POSITION_Y2 + POSITION_Y2 - POSITION_Y1 + POSITION_GAP)   {
    // Blue position
    if (mode == 1) {
      if (p.x > POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
        if (p.x < POSITION_X2 + POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
          position_2 = position_2 - increment;
        }
        else  {
          position_2 = position_2 + increment;
        }
      }
      position2Btn();
    }
    else if (mode == 2) {
      if (p.x > POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
        if (p.x < POSITION_X2 + POSITION_X2 + FIRST_BUTTON_DIFF + POSITION_GAP) {
          increment = increment - 1;
        }
        else  {
          increment = increment + 1;
        }
      }
      incrementBtn();
    }
  }
  else { //Botton row, next button
    if (p.x > NEXTBUTTON_X1) {
      if (position_1 > position_2) {
        position_2 = position_2 + 2;
      }
      else {
        position_1 = position_1 + 2;
      }
      position1Btn();
      position2Btn();
    }
    else { // Bottom row, settings button
      if (mode == 1) {
        mode = 2;
        setupSettingsScreen();
      }
      else {
        mode = 1;
        setupRecordingScreen();
      }
    }
  }
}

void liveStreamGyro() {
  unsigned long start_time = micros();
  for (int i = 0; i < numberGyroMeasurements; i++) {
    Serial.print(micros() - start_time);
    Serial.print(", ");
    get_accel_sample();
    get_accel2_sample();
    Serial.print(gyroSample);
    Serial.print(", ");
    Serial.println(gyro2Sample);
  }
}

void get_accel_sample() {
  // Read sample values
  imu.readAccel();
  String tempGyroSample = "";
  tempGyroSample += String(imu.calcAccel(imu.ax), 6);
  tempGyroSample += ", ";
  tempGyroSample += String(imu.calcAccel(imu.ay), 6);
  tempGyroSample += ", ";
  tempGyroSample += String(imu.calcAccel(imu.az), 6);
  tempGyroSample.toCharArray(gyroSample, 50);
  //Serial.println(tempGyroSample);
}

void get_accel2_sample() {
  // Read sample values
  imu2.readAccel();
  String tempGyroSample = "";
  tempGyroSample += String(imu2.calcAccel(imu2.ax), 6);
  tempGyroSample += ", ";
  tempGyroSample += String(imu2.calcAccel(imu2.ay), 6);
  tempGyroSample += ", ";
  tempGyroSample += String(imu2.calcAccel(imu2.az), 6);
  tempGyroSample.toCharArray(gyro2Sample, 50);
  //Serial.println(tempGyroSample);
}

// ----------------------------------------------------------------------------------------
//  Settings Screen
void setupSettingsScreen()
{
  tft.fillScreen(ILI9341_BLACK);
  exitButton();
  recordingTimeBtn();
  incrementBtn();
  fileListBtn();
}

void exitButton()
{
  tft.fillRect(SETTINGSBUTTON_X1, SETTINGSBUTTON_Y1, SETTINGSBUTTON_X2 - SETTINGSBUTTON_X1, SETTINGSBUTTON_Y2 - SETTINGSBUTTON_Y1, ILI9341_MAGENTA);
  tft.setCursor(SETTINGSBUTTON_X1 + 6 , SETTINGSBUTTON_Y1 + 30);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("EXIT");
}

void fileListBtn()
{
  tft.fillRect(GREENBUTTON_X1, GREENBUTTON_Y1, GREENBUTTON_X2 - GREENBUTTON_X1, GREENBUTTON_Y2 - GREENBUTTON_Y1, ILI9341_GREEN);
  tft.setCursor(GREENBUTTON_X1 + 6 , GREENBUTTON_Y1 + ((GREENBUTTON_Y2 - GREENBUTTON_Y1) / 2));
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(4);
  tft.println("File list");
}

void recordingTimeBtn()
{
  int width = POSITION_X2 - POSITION_X1;
  int height = POSITION_Y2 - POSITION_Y1;

  int x_position = POSITION_X1;
  tft.fillRect(POSITION_X1, POSITION_Y1, width + FIRST_BUTTON_DIFF, height, ILI9341_RED);
  // Label
  tft.setCursor(x_position + 4, POSITION_Y1 + 6);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.println("Rec:");

  // Recording time
  tft.setCursor(x_position + 4, POSITION_Y1 + 34);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.println(recording_time);

  // -
  x_position = x_position + width + FIRST_BUTTON_DIFF + POSITION_GAP;
  tft.fillRect(x_position, POSITION_Y1, width, height, ILI9341_RED);
  tft.setCursor(x_position - 35, POSITION_Y1 + 6);
  tft.setTextSize(8);
  tft.println(" - ");

  // +
  x_position = x_position + width + POSITION_GAP;
  tft.fillRect(x_position, POSITION_Y1, width, height, ILI9341_RED);
  tft.setCursor(x_position - 35, POSITION_Y1 + 6);
  tft.setTextSize(8);
  tft.println(" + ");
}

void incrementBtn()
{
  int width = POSITION_X2 - POSITION_X1;
  int height = POSITION_Y2 - POSITION_Y1;

  int x_position = POSITION_X1;
  int y_position = POSITION_Y1 + height + POSITION_GAP;
  tft.fillRect(POSITION_X1, y_position, width + FIRST_BUTTON_DIFF, height, ILI9341_BLUE);
  tft.setCursor(x_position + 4, y_position + 6);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.println("Inc:");

  // Recording time
  tft.setCursor(x_position + 4, y_position + 34);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.println(increment);

  // -
  x_position = x_position + width + FIRST_BUTTON_DIFF + POSITION_GAP;
  tft.fillRect(x_position, y_position, width, height, ILI9341_BLUE);
  tft.setCursor(x_position - 35, y_position + 6);
  tft.setTextSize(8);
  tft.println(" - ");

  // +
  x_position = x_position + width + POSITION_GAP;
  tft.fillRect(x_position, y_position, width, height, ILI9341_BLUE);
  tft.setCursor(x_position - 35, y_position + 6);
  tft.setTextSize(8);
  tft.println(" + ");
}


// ----------------------------------------------------------------------------------------
void listFiles() {
  //Not implemented yet!

  int entry_number = 1;
  tft.fillScreen(ILI9341_BLACK);
  while (true) {
    File dir = SD.open("/");
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files so draw the exit button
      exitButton();
      break;
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
    } else {
      // files have sizes, directories do not
      //tft.setCursor(1, entry_number * 3);
      //tft.setTextSize(2);
      //tft.println(entry.name());
      Serial.println(entry.name());
      entry_number = entry_number + 1;
    }
    entry.close();
  }
}

// ----------------------------------------------------------------------------------------
// Record Screen Setup
void setupRecordingScreen()
{
  tft.fillScreen(ILI9341_BLACK);
  recordBtn();
  settingsBtn();
  incBtn();
  position1Btn();
  position2Btn();
}

void recordBtn()
{
  tft.fillRect(GREENBUTTON_X1, GREENBUTTON_Y1, GREENBUTTON_X2 - GREENBUTTON_X1, GREENBUTTON_Y2 - GREENBUTTON_Y1, ILI9341_GREEN);
  tft.setCursor(GREENBUTTON_X1 + 6 , GREENBUTTON_Y1 + ((GREENBUTTON_Y2 - GREENBUTTON_Y1) / 2));
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(6);
  tft.println("RECORD");
}

void recordingBtn()
{
  tft.fillRect(GREENBUTTON_X1, GREENBUTTON_Y1, GREENBUTTON_X2 - GREENBUTTON_X1, GREENBUTTON_Y2 - GREENBUTTON_Y1, ILI9341_YELLOW);
  tft.setCursor(GREENBUTTON_X1 + 6 , GREENBUTTON_Y1 + ((GREENBUTTON_Y2 - GREENBUTTON_Y1) / 2));
}

void settingsBtn()
{
  tft.fillRect(SETTINGSBUTTON_X1, SETTINGSBUTTON_Y1, SETTINGSBUTTON_X2 - SETTINGSBUTTON_X1, SETTINGSBUTTON_Y2 - SETTINGSBUTTON_Y1, ILI9341_MAGENTA);
  tft.setCursor(SETTINGSBUTTON_X1 + 6 , SETTINGSBUTTON_Y1 + 30);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.println("SETTINGS");
}

void incBtn()
{
  tft.fillRect(NEXTBUTTON_X1, NEXTBUTTON_Y1, NEXTBUTTON_X2 - NEXTBUTTON_X1, NEXTBUTTON_Y2 - NEXTBUTTON_Y1, ILI9341_CYAN);
  tft.setCursor(NEXTBUTTON_X1 + 6 , NEXTBUTTON_Y1 + 20);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(5);
  tft.println("NEXT");
}

void position1Btn()
{
  int width = POSITION_X2 - POSITION_X1;
  int height = POSITION_Y2 - POSITION_Y1;

  int x_position = POSITION_X1;
  tft.fillRect(POSITION_X1, POSITION_Y1, width + FIRST_BUTTON_DIFF, height, ILI9341_RED);
  tft.setCursor(x_position + 4, POSITION_Y1 + 12);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(5);
  tft.println(position_1);

  // -
  x_position = x_position + width + FIRST_BUTTON_DIFF + POSITION_GAP;
  tft.fillRect(x_position, POSITION_Y1, width, height, ILI9341_RED);
  tft.setCursor(x_position - 35, POSITION_Y1 + 6);
  tft.setTextSize(8);
  tft.println(" - ");

  // +
  x_position = x_position + width + POSITION_GAP;
  tft.fillRect(x_position, POSITION_Y1, width, height, ILI9341_RED);
  tft.setCursor(x_position - 35, POSITION_Y1 + 6);
  tft.setTextSize(8);
  tft.println(" + ");
}

void position2Btn()
{
  int width = POSITION_X2 - POSITION_X1;
  int height = POSITION_Y2 - POSITION_Y1;

  int x_position = POSITION_X1;
  int y_position = POSITION_Y1 + height + POSITION_GAP;
  tft.fillRect(POSITION_X1, y_position, width + FIRST_BUTTON_DIFF, height, ILI9341_BLUE);
  tft.setCursor(x_position + 4, y_position + 12);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(5);
  tft.println(position_2);

  // -
  x_position = x_position + width + FIRST_BUTTON_DIFF + POSITION_GAP;
  tft.fillRect(x_position, y_position, width, height, ILI9341_BLUE);
  tft.setCursor(x_position - 35, y_position + 6);
  tft.setTextSize(8);
  tft.println(" - ");

  // +
  x_position = x_position + width + POSITION_GAP;
  tft.fillRect(x_position, y_position, width, height, ILI9341_BLUE);
  tft.setCursor(x_position - 35, y_position + 6);
  tft.setTextSize(8);
  tft.println(" + ");
}
// ----------------------------------------------------------------------------------------

void logData()
{
  recordingBtn();
  String filename = String(position_1) + "_" + String(position_2) + "_" + String(position_counter);
  char charfilename[filename.length() + 1];
  Serial.println(filename.length() + 1);
  filename.toCharArray(charfilename, filename.length() + 1);
  position_counter = position_counter + 1;
  dataFile = SD.open(charfilename, FILE_WRITE);
  Serial.print("Logging file: ");
  Serial.print(filename);
  Serial.print(" / ");
  Serial.print(charfilename);
  Serial.print(", samples: ");
  Serial.print(num_samples);
  Serial.print("...");
  unsigned long start_time = micros();

  for (int i = 0; i < num_samples; i++)  {
    get_accel();
    dataFile.print(micros() - start_time);
    dataFile.print(", ");
    dataFile.println(gyroSample);
    //Serial.println(gyroSample);
  }
  dataFile.close();
  Serial.println("Complete");
  recordBtn();

  get_accel_sample();
  get_accel2_sample();
  Serial.println(gyroSample);
  Serial.println(gyro2Sample);

  // Resolution of screen in 240 x 320, so we can plot 320 values from the data
  //  int sampinterval = round(num_samples/320);
  //  Serial.print("Plot interval: ");
  //  Serial.println(sampinterval);
  //
  //
  //  tft.fillScreen(ILI9341_BLACK);
  //  tft.drawPixel(10, 10, ILI9341_GREEN);
  //
  //  delay(2000);
  //  setupRecordingScreen();
}

void get_accel() {
  imu.readAccel();
  imu2.readAccel();

  String tempGyroSample = "";
  //tempGyroSample += String(imu.calcAccel(imu.ax), 6);
  //tempGyroSample += ", ";
  tempGyroSample += String(imu.calcAccel(imu.ay), 6);
  tempGyroSample += ", ";
  //tempGyroSample += String(imu.calcAccel(imu.az), 6);
  //tempGyroSample += ", ";
  //tempGyroSample += String(imu2.calcAccel(imu2.ax), 6);
  //tempGyroSample += ", ";
  tempGyroSample += String(imu2.calcAccel(imu2.ay), 6);
  //tempGyroSample += ", ";
  //tempGyroSample += String(imu2.calcAccel(imu2.az), 6);

  tempGyroSample.toCharArray(gyroSample, 80); // If the number of characters copied is too small then all the values will not be included.
}

void printAccel()
{
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.ax), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.ay), 2);
  Serial.print(", ");
  Serial.print(imu2.calcAccel(imu2.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif
}

// BMP section

#define BUFFPIXEL 20

void bmpDraw(char *filename, int16_t x, int16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col, x2, y2, bx1, by1;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if ((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if (read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if (read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if (bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        x2 = x + bmpWidth  - 1; // Lower-right corner
        y2 = y + bmpHeight - 1;
        if ((x2 >= 0) && (y2 >= 0)) { // On screen?
          w = bmpWidth; // Width/height of section to load/display
          h = bmpHeight;
          bx1 = by1 = 0; // UL coordinate in BMP file
          if (x < 0) { // Clip left
            bx1 = -x;
            x   = 0;
            w   = x2 + 1;
          }
          if (y < 0) { // Clip top
            by1 = -y;
            y   = 0;
            h   = y2 + 1;
          }
          if (x2 >= tft.width())  w = tft.width()  - x; // Clip right
          if (y2 >= tft.height()) h = tft.height() - y; // Clip bottom

          // Set TFT address window to clipped image bounds
          tft.startWrite(); // Requires start/end transaction now
          tft.setAddrWindow(x, y, w, h);

          for (row = 0; row < h; row++) { // For each scanline...

            // Seek to start of scan line.  It might seem labor-
            // intensive to be doing this on every line, but this
            // method covers a lot of gritty details like cropping
            // and scanline padding.  Also, the seek only takes
            // place if the file position actually needs to change
            // (avoids a lot of cluster math in SD library).
            if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
              pos = bmpImageoffset + (bmpHeight - 1 - (row + by1)) * rowSize;
            else     // Bitmap is stored top-to-bottom
              pos = bmpImageoffset + (row + by1) * rowSize;
            pos += bx1 * 3; // Factor in starting column (bx1)
            if (bmpFile.position() != pos) { // Need seek?
              tft.endWrite(); // End TFT transaction
              bmpFile.seek(pos);
              buffidx = sizeof(sdbuffer); // Force buffer reload
              tft.startWrite(); // Start new TFT transaction
            }
            for (col = 0; col < w; col++) { // For each pixel...
              // Time to read more pixel data?
              if (buffidx >= sizeof(sdbuffer)) { // Indeed
                tft.endWrite(); // End TFT transaction
                bmpFile.read(sdbuffer, sizeof(sdbuffer));
                buffidx = 0; // Set index to beginning
                tft.startWrite(); // Start new TFT transaction
              }
              // Convert pixel from BMP to TFT format, push to display
              b = sdbuffer[buffidx++];
              g = sdbuffer[buffidx++];
              r = sdbuffer[buffidx++];
              tft.writePixel(tft.color565(r, g, b));
            } // end pixel
          } // end scanline
          tft.endWrite(); // End last TFT transaction
        } // end onscreen
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if (!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
