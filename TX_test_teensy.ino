#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <SD.h>

#include <PulsePosition.h>

#define ANALOG_READBAT_PIN A17

#define ANALOG_READA_PIN A0
#define ANALOG_READB_PIN A1
#define ANALOG_READC_PIN A2
#define ANALOG_READD_PIN A3
#define ANALOG_READE_PIN A4
#define ANALOG_READF_PIN A5

#define PIN_SW_A      33
#define PIN_SW_B      34
#define PIN_SW_C_LOW  35
#define PIN_SW_C_HI   36
#define PIN_SW_D_LOW  37
#define PIN_SW_D_HI   38

#define NAV_A_C    40
#define NAV_A_L    24
#define NAV_A_R    25
#define NAV_A_U    26
#define NAV_A_D    27

#define NAV_B_C    41
#define NAV_B_L    28
#define NAV_B_R    29
#define NAV_B_U    30
#define NAV_B_D    31

#define PPM_OUT_PIN     6

#define MODULE_PWR_PIN  32
#define LED_PWR_PIN     13

#define trim_label0 "STR"
#define trim_label1 "THR"
#define trim_label2 "AUX1"
#define trim_label3 "AUX2"

#define input_label0 "STR"
#define input_label1 "THR"
#define input_label2 "ThH"
#define input_label3 "ThV"
#define input_label4 "P1"
#define input_label5 "P2"
#define input_label6 "SW1"
#define input_label7 "SW2"
#define input_label8 "SW3"
#define input_label9 "SW4"
#define input_label10 "P3"
#define input_label11 "P4"
#define input_label12 "SW5"
#define input_label13 "SW6"

PulsePositionOutput PPMoutput;

struct calibration_data {
  int low;
  int mid;
  int high;
};

struct mix_data {
  int channel;
  int input;
  int weight;
  int dir;
};

struct channel_data {
  mix_data mix[3];
};

struct model_data {
  char name[12];
  calibration_data cal[8];
  int trim [4];
  int telemetry;
  int channel_count;
  channel_data channels[16];
};

struct model_data *model_info = (struct model_data*)malloc( sizeof(model_data) );


int stat = sizeof(channel_data);

char trim_label[4][10] {trim_label0, trim_label1, trim_label2, trim_label3};

char input_label[14][10] {input_label0, input_label1, input_label2, input_label3, input_label4, input_label5, input_label6, input_label7, input_label8, input_label9, input_label10, input_label11, input_label12, input_label13};

int nav_a_array[7];
int nav_b_array[7];

int trim_array[5] = {0, 0, 0, 0, 0};
//array of calibration values for low, mid, high on analog inputs  -- ultimately these need to be saved in eeprom or SD card
int cal_array[8][3];
int lcd_active = LOW;
unsigned long lcd_counter = 0;  // 15 millisecond timer for lcd active

int sigArray[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int PPMArray[17] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
int PPMIndex = 0;
int PPMState = HIGH;
long PPMtotal = 19800;

int EXIT_key = false;
int MENU_key = false;

int PWR_value = 0;

int draw_state = 0;
int mix_channel = 0;

int lcount = 0;

int sd_available = 1;
char modelName[22] = "MODEL01";

// stm32 oled 1309 setup
//oled pins: 1 = cs, 2 = dc, 3 = reset, 4 = data, 5 = clock, 6 = VCC, 7 = GND
//U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R1, /* clock=*/ PB12, /* data=*/ PB13, /* cs=*/ PA8, /* dc=*/ PB15, /* reset=*/ PB14);
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/ PB12, /* dc=*/ PA8, /* reset=*/ PB14);

// stm32 st7565 setup
//U8G2_ST7565_64128N_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/ PB12, /* dc=*/ PA8, /* reset=*/ PB14);
// STM32 HW pins: MISO = PB14, MOSI = PB15, SCK = PB13, cs/ss = PB12, dc = PA8, reset = PA15

// teensy st7565 setup
//U8G2_ST7565_64128N_F_4W_SW_SPI u8g2(U8G2_R1, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
U8G2_ST7565_64128N_F_4W_HW_SPI u8g2(U8G2_R1, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

//LCD pins 7 = clock, 8 = data, 9 = VCC, 10 = GND, 11 = Backlight, 12 = cs/ss, 13 = reset, 14 = dc

void u8g2_prepare(void) {
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void hide_blot(void) {
//  u8g2.setDrawColor(0);
//  u8g2.drawHLine(0, 35, 4);
//  u8g2.drawHLine(0, 36, 4);
//  u8g2.setDrawColor(1);
}

void draw_blot(void) {
  char state_str[10] = "CH";
  
  u8g2.setDrawColor(1);
//  u8g2.drawHLine(0, 35, 4);
//  u8g2.drawHLine(0, 36, 4);

  sprintf(state_str,"screen: %d", draw_state);
  u8g2.drawStr(0, 36, state_str);
}

void u8g2_draw_limits(uint8_t top, uint8_t channel) {
  char channel_str[10] = "CH";
  char value_str[10] = "Val:";
  char min_str[10] = "Min: -100";
  char max_str[10] = "Max: +100";
  char mid_str[10] = "Mid: 0";
  char dir_str[10] = "Dir: NOR";
  int val;

  sprintf(channel_str,"CH%d", channel);
  val = PPMArray[channel-1];
  val = val + 300;
  sprintf(value_str, "Val: %d", val);
  
  u8g2.drawStr( 0, top, channel_str);
  u8g2.drawStr(0, top+8, value_str);
  u8g2.drawStr(0, top+16, min_str);
  u8g2.drawStr(0, top+24, max_str);
  u8g2.drawStr(0, top+32, mid_str);
  u8g2.drawStr(0, top+40, dir_str);
}

void u8g2_draw_channel(uint8_t top, uint8_t channel) {
  char channel_str[10] = "CH";
  int la, lb, wa, wb, val;

  sprintf(channel_str,"CH%d", channel);
  val = PPMArray[channel-1];
  val = val -1500;
  val = val / 20;

  la = 39;
  wa = abs(val);

  if (val < 0) {
    la = 39+val;
  }
  
  u8g2.drawStr( 0, top, channel_str);
  u8g2.drawFrame(15, top, 49, 7);
  u8g2.drawVLine(39, top, 7);
  u8g2.drawVLine(27, top+3, 3);
  u8g2.drawVLine(51, top+3, 3);
  u8g2.drawHLine(la, top+4, wa);
  u8g2.drawHLine(la, top+5, wa);
}

void u8g2_draw_input(uint8_t top, uint8_t input) {
  String input_str;
  int la, lb, wa, wb, val;
  
  val = sigArray[input-1];
  val = val -1500;
  val = val / 20;

  la = 39;
  wa = abs(val);

  if (val < 0) {
    la = 39+val;
  }
  
  u8g2.drawStr( 0, top, input_label[input-1]);
  u8g2.drawFrame(15, top, 49, 7);
  u8g2.drawVLine(39, top, 7);
  u8g2.drawVLine(27, top+3, 3);
  u8g2.drawVLine(51, top+3, 3);
  u8g2.drawHLine(la, top+4, wa);
  u8g2.drawHLine(la, top+5, wa);
}

void u8g2_draw_trim(uint8_t top, uint8_t channel) {

  u8g2.drawStr( 0, top-3, trim_label[channel-1]);
  u8g2.drawHLine(15, top, 64);
  u8g2.drawVLine(39, top-2, 5);
  u8g2.drawFrame(37+trim_array[channel-1], top-2, 5, 5);

}

void draw_voltage(uint8_t left, uint8_t top) {
  char bat_str[18] = "                ";

//  (3.3 / 4095);
  float voltage = PWR_value * (3.3 / 4095);
  
  sprintf(bat_str," %d.%02dv     %d,%02dv", (int)voltage, (int)(voltage*100)%100, 0, 0);
  
  u8g2.drawStr( left, top, bat_str);
}

void u8g2_frame(int draw_state) {

// current model name
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.drawStr( 0, 0, "Vehicle 001");

// faked telemetry
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.setFontRefHeightExtendedText();

  if (MENU_key == true) {
    draw_blot();
  }
//tx and rx voltage
//u8g2.drawStr( 0, 16, " 8.20v     4.90v");
  draw_voltage(0, 16);
//tx and rx rssi telemetry
//  u8g2.drawStr( 0, 24, "TX=100    RX=100");
//flight pack telemetry cell voltages
//  u8g2.drawStr( 0, 32, "4.00  3.99  3.99");

  if (draw_state == 0) {
  // channel 1 trim
    u8g2_draw_trim(49, 1);
  // channel 2 trim
    u8g2_draw_trim(57, 2);
    
    u8g2.drawHLine(0, 63, 64);
    
    u8g2_draw_channel(65, 1);
    u8g2_draw_channel(73, 2);
    u8g2_draw_channel(81, 3);
    u8g2_draw_channel(89, 4);
    u8g2_draw_channel(97, 5);
    u8g2_draw_channel(105, 6);
    u8g2_draw_channel(113, 7);
    u8g2_draw_channel(121, 8);
  }

  if (draw_state == 1) {
    u8g2.drawHLine(0, 47, 64);
    
    u8g2_draw_input(49, 1);
    u8g2_draw_input(57, 2);
    u8g2_draw_input(65, 3);
    u8g2_draw_input(73, 4);
    u8g2_draw_input(81, 5);
    u8g2_draw_input(89, 6);
    u8g2_draw_input(97, 7);
    u8g2_draw_input(105, 8);
    u8g2_draw_input(113, 9);
    u8g2_draw_input(121, 10);
  }

  if (draw_state == 2) {
    u8g2.drawHLine(0, 63, 64);
    
    u8g2.drawStr(0, 65, "CH1  + Str 100%");
    u8g2.drawStr(0, 73, "CH2  + Thr 100%");
    u8g2.drawStr(0, 81, "CH3  + ThH 100%");
    u8g2.drawStr(0, 89, "CH4  + ThV 100%");
    u8g2.drawStr(0, 97, "CH5  + P1  100%");
    u8g2.drawStr(0, 105, "CH6  + P2  100%");
    u8g2.drawStr(0, 113, "CH7  + SW1 100%");
    u8g2.drawStr(0, 121, "CH8  + SW3 100%");

    if (mix_channel != 0) {
      u8g2.setDrawColor(2);

      int top = 57+(mix_channel*8);
      u8g2.drawBox(0, top, 64, 8);
      u8g2.setDrawColor(1);
    }
  }
  
  if (draw_state == 11) {
    u8g2.drawHLine(0, 47, 64);
    
    u8g2_draw_limits(49, mix_channel);
  }
}

void draw(void) {
  u8g2_prepare();

  u8g2_frame(draw_state);
}

void read_card_data(int model_ID) {
  // set up variables using the SD utility library functions:
  File dataFile;
  char filename[12];
  char buf;
  
  const int SDchipSelect = BUILTIN_SDCARD;
  
  sprintf(filename,"MODEL%d.DAT", model_ID);
  
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  //if (!card.init(SPI_HALF_SPEED, SDchipSelect)) {

  if (SD.begin(SDchipSelect)) {
    // card appears to be available
    if (SD.exists(filename)) {
      //file exists so read it into our model record
      dataFile = SD.open(filename);
      if (dataFile) {
        buf = char(model_info);
        while (dataFile.available()) {
          buf = dataFile.read();
          buf = buf+1;
        }
        dataFile.close();
      }
    } else {
      // file doesn't exist, so write a default model
      buf = char(model_info);
      dataFile = SD.open(filename, FILE_WRITE);
      for(int i = 0; i < sizeof(model_data); i++) {
        dataFile.write(buf);
        buf = buf+1;
      }
      dataFile.close();
    }
  }

  
  
}

void read_inputs() {

  int analogInt;
  int digitalInt;
  int lcd_on = 0;
  int fix_draw_state = 0;
  
  analogInt = analogRead(ANALOG_READBAT_PIN);
  PWR_value = analogInt;

  analogInt = analogRead(ANALOG_READA_PIN);
  analogInt = analogInt/4;
  analogInt = analogInt + 1000;
  sigArray[0] = analogInt;
  
  analogInt = analogRead(ANALOG_READB_PIN);
  analogInt = analogInt/4;
  analogInt = analogInt + 1000;
  sigArray[1] = analogInt;
  
  analogInt = analogRead(ANALOG_READC_PIN);
  analogInt = analogInt/4;
  analogInt = analogInt + 1000;
  sigArray[2] = analogInt;
  
  analogInt = analogRead(ANALOG_READD_PIN);
  analogInt = analogInt/4;
  analogInt = analogInt + 1000;
  sigArray[3] = analogInt;
  
  analogInt = analogRead(ANALOG_READE_PIN);
  analogInt = analogInt/4;
  analogInt = analogInt + 1000;
  sigArray[4] = analogInt;
  
  analogInt = analogRead(ANALOG_READF_PIN);
  analogInt = analogInt/4;
  analogInt = analogInt + 1000;
  sigArray[5] = analogInt;
  
  // 2 position switch A
  digitalInt = digitalRead(PIN_SW_A);
  analogInt = 1000;
  if (digitalInt == HIGH) {
    analogInt = 2000;
  }
  sigArray[6] = analogInt;

  // 2 position switch B
  digitalInt = digitalRead(PIN_SW_B);
  analogInt = 1000;
  if (digitalInt == HIGH) {
    analogInt = 2000;
  }
  sigArray[8] = analogInt;

  // 3 position switch A left side
  analogInt = 1500;
  digitalInt = digitalRead(PIN_SW_C_LOW);
  if (digitalInt == HIGH) {
    analogInt = 1000;
  }
  // 3 position switch A right side
  digitalInt = digitalRead(PIN_SW_C_HI);
  if (digitalInt == HIGH) {
    analogInt = 2000;
  }
  sigArray[7] = analogInt;

  // 3 position switch B left side
  analogInt = 1500;
  digitalInt = digitalRead(PIN_SW_D_LOW);
  if (digitalInt == HIGH) {
    analogInt = 1000;
  }
  // 3 position switch B right side
  digitalInt = digitalRead(PIN_SW_D_HI);
  if (digitalInt == HIGH) {
    analogInt = 2000;
  }
  sigArray[9] = analogInt;
  
  // read the two 5 pin Nav switches
  nav_a_array[0] = digitalRead(NAV_A_C);
  nav_a_array[1] = digitalRead(NAV_A_L);
  nav_a_array[2] = digitalRead(NAV_A_R);
  nav_a_array[3] = digitalRead(NAV_A_U);
  nav_a_array[4] = digitalRead(NAV_A_D);
  
  nav_b_array[0] = digitalRead(NAV_B_C);
  nav_b_array[1] = digitalRead(NAV_B_L);
  nav_b_array[2] = digitalRead(NAV_B_R);
  nav_b_array[3] = digitalRead(NAV_B_U);
  nav_b_array[4] = digitalRead(NAV_B_D);
  
  if (nav_a_array[1] == HIGH) {
    trim_array[1] = trim_array[1]+1;
 //   lcd_on = 1;
  }
  if (nav_a_array[2] == HIGH) {
    trim_array[0] = trim_array[0]-1;
 //   lcd_on = 1;
  }
  if (nav_a_array[3] == HIGH) {
    trim_array[1] = trim_array[1]-1;
 //   lcd_on = 1;
  }
  if (nav_a_array[4] == HIGH) {
    trim_array[0] = trim_array[0]+1;
 //   lcd_on = 1;
  }

  if (trim_array[0] > 25){
    trim_array[0] = 25;
  }
  if (trim_array[0] < -25){
    trim_array[0] = -25;
  }
  if (trim_array[1] > 25){
    trim_array[1] = 25;
  }
  if (trim_array[1] < -25){
    trim_array[1] = -25;
  }
  
  if (nav_a_array[0] == HIGH) {
    EXIT_key = true;
    MENU_key = false;
    mix_channel = 0;
    lcd_on = 1;
    hide_blot();
  }

  if (nav_b_array[0] == HIGH) {
    if (MENU_key == true) {
      if (mix_channel > 0) {
        fix_draw_state = 11;
      }
    }
    MENU_key = true;
    EXIT_key = false;
    lcd_on = 1;
    draw_blot();
  }

  if (MENU_key == false) {
    if (nav_b_array[1] == HIGH) {
      trim_array[3] = trim_array[3]+1;
 //     lcd_on = 1;
    }
    if (nav_b_array[2] == HIGH) {
      trim_array[2] = trim_array[2]-1;
 //     lcd_on = 1;
    }
    if (nav_b_array[3] == HIGH) {
      trim_array[3] = trim_array[3]-1;
 //     lcd_on = 1;
    }
    if (nav_b_array[4] == HIGH) {
      trim_array[2] = trim_array[2]+1;
//      lcd_on = 1;
    }
    
    if (trim_array[2] > 25){
      trim_array[2] = 25;
    }
    if (trim_array[2] < -25){
      trim_array[2] = -25;
    }
    if (trim_array[3] > 25){
      trim_array[3] = 25;
    }
    if (trim_array[3] < -25){
      trim_array[3] = -25;
    }
  }
  else {
  //left on b stick
    if (nav_b_array[1] == HIGH) {
      draw_state = draw_state - 1;
      lcd_on = 1;
    }

    //right on b stick
    if (nav_b_array[3] == HIGH) {
      draw_state = draw_state + 1;
      lcd_on = 1;
    }
  //up on b stick
    if (nav_b_array[2] == HIGH) {
      mix_channel = mix_channel - 1;
      lcd_on = 1;
    }

    //down on b stick
//    if (nav_b_array[4] == HIGH) {
//      mix_channel = mix_channel + 1;
//      lcd_on = 1;
//    }
  }
  
  if (draw_state < 0) {
    draw_state = 5;
  }
  if (draw_state > 5) {
    if (draw_state > 10) {
      fix_draw_state = draw_state;
    }
    draw_state = 0;
  }
  
  if (mix_channel < 0) {
    mix_channel = 8;
  }
  if (mix_channel > 8) {
    mix_channel = 1;
  }
  
  if (fix_draw_state > 0) {
    draw_state = fix_draw_state;
  }
  
  for (int i = 0; i < 4; i++){
    //currently only maps low and high, doesn't yet use mid
    //sigArray[i] = map(sigArray[i], cal_array[i][0], cal_array[i][2], 1000, 2000);
    sigArray[i] = sigArray[i] + (4 * trim_array[i]);
  }
  
  //for now, copy first eight signals (inputs) to the same PPMs (outputs)
  for(int i = 0; i < 8; i++){
    PPMArray[i] = sigArray[i];
    PPMoutput.write(i, PPMArray[i]);
  }

  if (lcd_on == 1) {
      lcd_active = HIGH;
      lcd_counter = millis()+15000;
      digitalWrite(LED_PWR_PIN, lcd_active);
  }
}

void setup() {
  File modelFile;
  int i;

  Serial.begin(9600);

//  SPI.setModule(2);
  
//  pinMode(ANALOG_READA_PIN, INPUT_ANALOG);
//  pinMode(ANALOG_READB_PIN, INPUT_ANALOG);
//  pinMode(ANALOG_READC_PIN, INPUT_ANALOG);
//  pinMode(ANALOG_READD_PIN, INPUT_ANALOG);
//  pinMode(ANALOG_READE_PIN, INPUT_ANALOG);
//  pinMode(ANALOG_READF_PIN, INPUT_ANALOG);

  cal_array[0][0] = 1232;
  cal_array[0][1] = 1574;
  cal_array[0][2] = 2014;

  cal_array[1][0] = 1231;
  cal_array[1][1] = 1584;
  cal_array[1][2] = 1975;

  cal_array[2][0] = 1359;
  cal_array[2][1] = 1530;
  cal_array[2][2] = 1850;

  cal_array[3][0] = 1333;
  cal_array[3][1] = 1554;
  cal_array[3][2] = 1859;

  cal_array[4][0] = 1000;
  cal_array[4][1] = 1500;
  cal_array[4][2] = 2000;

  cal_array[5][0] = 1000;
  cal_array[5][1] = 1500;
  cal_array[5][2] = 2000;

  pinMode(NAV_A_C, INPUT_PULLDOWN);
  pinMode(NAV_A_L, INPUT_PULLDOWN);
  pinMode(NAV_A_R, INPUT_PULLDOWN);
  pinMode(NAV_A_U, INPUT_PULLDOWN);
  pinMode(NAV_A_D, INPUT_PULLDOWN);
  
  pinMode(NAV_B_C, INPUT_PULLDOWN);
  pinMode(NAV_B_L, INPUT_PULLDOWN);
  pinMode(NAV_B_R, INPUT_PULLDOWN);
  pinMode(NAV_B_U, INPUT_PULLDOWN);
  pinMode(NAV_B_D, INPUT_PULLDOWN);
  
  pinMode(PIN_SW_A, INPUT_PULLDOWN);
  pinMode(PIN_SW_B, INPUT_PULLDOWN);
  
  pinMode(PIN_SW_C_LOW, INPUT_PULLDOWN);
  pinMode(PIN_SW_C_HI, INPUT_PULLDOWN);
  
  pinMode(PIN_SW_D_LOW, INPUT_PULLDOWN);
  pinMode(PIN_SW_D_HI, INPUT_PULLDOWN);
  
  pinMode(MODULE_PWR_PIN, OUTPUT);
  pinMode(LED_PWR_PIN, OUTPUT);
  
  PPMoutput.begin(PPM_OUT_PIN);
  for(i=0;i<8;i++) {
    PPMoutput.write(i, PPMArray[i]);
  }

  //figure out how to read from built in microSD card 
  /*
  if (!SD.begin(PB11)) {
    sd_available = 0;
  } 
  else {
    //read the data from the placeholder file to kickstart model data load
    if (SD.exists("current.mdl")) {
      modelFile = SD.open("current.mdl");
      if (modelFile) {
        modelFile.read(modelName, 10);
        modelFile.close();
      }
    } else {
    // create a new model file
      modelFile = SD.open("current.mdl", FILE_WRITE);
      if (modelFile) {
        modelFile.print(modelName);
        modelFile.close();
      }
    }
  }
  */

  u8g2.begin();
  
  read_inputs();

  // put your main code here, to run repeatedly:
  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();
  
  digitalWrite(MODULE_PWR_PIN, HIGH);
}

void loop() {
  unsigned long time_now = 0;
  
  time_now = millis();

  //here we will do analog read/digital read of assigned input pins, and set 8 ppm values or UI draw states accordingly
  
  read_inputs();
  
  if (lcd_active == HIGH) {
    if (lcd_counter < millis()) {
      lcd_active = LOW;
      lcd_counter = 0;
      digitalWrite(LED_PWR_PIN, lcd_active);
    }
  }

  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();

  //go through the loop 50 times a second
  while(millis() < time_now + 20){
    //wait approx. [period] ms
  }
  
  lcount = lcount+1;
  
  if (lcount == 50) {
    lcount = 0;

 /*   
    Serial.print("trim0 ");
    Serial.println(trim_array[0]);
    Serial.print("trim1 ");
    Serial.println(trim_array[1]);
    Serial.print("trim2 ");
    Serial.println(trim_array[2]);
    Serial.print("trim3 ");
    Serial.println(trim_array[3]);
    Serial.println("");
    
    Serial.print("na0 ");
    Serial.println(nav_a_array[0]);
    Serial.print("na1 ");
    Serial.println(nav_a_array[1]);
    Serial.print("na2 ");
    Serial.println(nav_a_array[2]);
    Serial.print("na3 ");
    Serial.println(nav_a_array[3]);
    Serial.print("na4 ");
    Serial.println(nav_a_array[4]);
    Serial.println("");
 */
    
    Serial.print("nb0 ");
    Serial.println(nav_b_array[0]);
    Serial.print("nb1 ");
    Serial.println(nav_b_array[1]);
    Serial.print("nb2 ");
    Serial.println(nav_b_array[2]);
    Serial.print("nb3 ");
    Serial.println(nav_b_array[3]);
    Serial.print("nb4 ");
    Serial.println(nav_b_array[4]);
    Serial.println("");
/*
    Serial.print("a0 ");
    Serial.println(sigArray[0]);
    Serial.print("a1 ");
    Serial.println(sigArray[1]);
    Serial.print("a2 ");
    Serial.println(sigArray[2]);
    Serial.print("a3 ");
    Serial.println(sigArray[3]);
    Serial.print("a4 ");
    Serial.println(sigArray[4]);
    Serial.print("a5 ");
    Serial.println(sigArray[5]);

/*
    Serial.print("a6 ");
    Serial.println(sigArray[6]);
    Serial.print("a7 ");
    Serial.println(sigArray[7]);
    Serial.print("a8 ");
    Serial.println(sigArray[8]);
    Serial.print("a9 ");
    Serial.println(sigArray[9]);
    Serial.println("");
 

    for (int i = 0; i < 16; i++){
      Serial.print("Mux");
      Serial.print(i);
      Serial.print(":");
      Serial.println(mux_array[0]);
    }
    Serial.println(" ");

*/    
  }
}
