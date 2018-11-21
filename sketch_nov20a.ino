#include <Wire.h>
#include <Adafruit_ADS1015.h>               // ADS1015,1115

byte contrast = 35;                         // LCD contrast (0-63)

Adafruit_ADS1115 ads(0x48);                 // 16-bit version

//BME280 parameters
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;
uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;
 int8_t  dig_H1;
 int16_t dig_H2;
 int8_t  dig_H3;
 int16_t dig_H4;
 int16_t dig_H5;
 int8_t  dig_H6;

void setup() {
  Serial.begin(115200);

  Wire.begin(4,5);                          // SDA,SCL
  
  pinMode(14, OUTPUT);                      // Dust sensor enable pin
  digitalWrite(14, LOW);
  
  ads.setGain(GAIN_TWOTHIRDS);              // 2/3x gain +/-6.144V
  ads.begin();                              // 1 bit=0.1875mV

  init_BME();

  init_LCD();
}

void loop()
{
  char s[10];

  // Dust sensor

  digitalWrite(14, HIGH);  // Turn ON Dust sensor
  uint16_t v = 0;
  for(int i=0; i<500; i++) {
    uint16_t v0 = ads.readADC_SingleEnded(0); // A0 Read
    if(v < v0) v = v0;
  }
  digitalWrite(14, LOW);   // Turn OFF Dust sensor
  dtostrf((double)v * 0.1875/1000, 3, 1, s);
  strcat(s, "v");
  lcd_setCursor(0, 0);
  lcd_printStr(s); // Show in LCD


  // BME280
  
  readData();
  double temp = (double)calibration_T(temp_raw) / 100;
  double pres = (double)calibration_P(pres_raw) / 100;
  double humi = (double)calibration_H(hum_raw) / 1024;

  dtostrf(temp, 2, 0, s);
  strcat(s, "c");
  lcd_setCursor(0, 1);
  lcd_printStr(s); // Show in LCD

  dtostrf(humi, 2, 0, s);
  strcat(s, "%");
  lcd_setCursor(4, 1);
  lcd_printStr(s); // Show in LCD
  
  dtostrf(pres, 4, 0, s);
  lcd_setCursor(4, 0);
  lcd_printStr(s); // Show in LCD

  delay(1000);
}

///////////////////
// BME280 functions

void init_BME()
{
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off 
  uint8_t spi3w_en = 0;           //3-wire SPI Disable
  
  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;
  
  writeReg(0xF2,ctrl_hum_reg);
  writeReg(0xF4,ctrl_meas_reg);
  writeReg(0xF5,config_reg);
  readTrim();                    //
}

void readTrim()
{
    uint8_t data[32],i=0;
    Wire.beginTransmission(0x76);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(0x76,24);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    
    Wire.beginTransmission(0x76);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(0x76,1);
    data[i] = Wire.read();
    i++;
    
    Wire.beginTransmission(0x76);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(0x76,7);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];   
}
void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(0x76);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}


void readData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(0x76);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(0x76 ,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{
    
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}

///////////////////
// LCD functions

void init_LCD()
{
  delay(500);
  lcd_cmd(0b00111000); // function set
  lcd_cmd(0b00111001); // function set
  lcd_cmd(0b00000100); // EntryModeSet
  lcd_cmd(0b00010100); // interval osc
  lcd_cmd(0b01110000 | (contrast & 0xF)); // contrast Low
  lcd_cmd(0b01011100 | ((contrast >> 4) & 0x3)); // contast High/icon/power
  lcd_cmd(0b01101100); // follower control
  delay(200);
  lcd_cmd(0b00111000); // function set
  lcd_cmd(0b00001100); // Display On
  lcd_cmd(0b00000001); // Clear Display
  delay(2);
}

void lcd_cmd(byte x) {
  Wire.beginTransmission(0x3e);
  Wire.write(0b00000000); // CO = 0,RS = 0
  Wire.write(x);
  Wire.endTransmission();
}
 
void lcd_contdata(byte x) {
  Wire.write(0b11000000); // CO = 1, RS = 1
  Wire.write(x);
}
 
void lcd_lastdata(byte x) {
  Wire.write(0b01000000); // CO = 0, RS = 1
  Wire.write(x);
}
 
// 文字の表示
void lcd_printStr(const char *s) {
  Wire.beginTransmission(0x3e);
  while (*s) {
    if (*(s + 1)) {
      lcd_contdata(*s);
    } else {
      lcd_lastdata(*s);
    }
    s++;
  }
  Wire.endTransmission();
}
 
// 表示位置の指定
void lcd_setCursor(byte x, byte y) {
  lcd_cmd(0x80 | (y * 0x40 + x));
}
