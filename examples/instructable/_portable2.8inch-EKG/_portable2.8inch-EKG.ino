/*--------------------------------------------------------------
  Program:       voltmeter

  Description:   DC voltmeter with voltage  displayed
                  on Color TFT LCD to 1 decimal place
  
  Hardware:      Arduino NANO with voltage divider on A0.
                 TFT LCD connected 
                
  Software:      Developed using Arduino 1.0.3 software
                

  Date:          10 March 2014
 
  Author:        johnag    
--------------------------------------------------------------*/
int y1[235];
int x1;
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPI.h>
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);



// voltage divider calibration value
#define Dv1    10.935

// ADC reference voltage / calibration value
#define VREF    5.03



float V1 = {0.00}; 



void setup()
{
  Serial.begin(9600);
   Serial.println(F("Using Adafruit 2.8\" TFT Breakout Board Pinout"));

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

  tft.reset();

  uint16_t identifier = tft.readID();

  tft.begin(identifier); 
   tft.fillScreen(BLACK); //  clear screen
   
}

void loop()
{
  tft.fillScreen(BLACK);
   tft.drawRect(5, 5, 235,315, RED);
 /* tft.drawRoundRect(4,150, 120, 100, 5,GREEN);
  tft.drawLine(4, 170, 120, 170, GREEN);
  tft.drawLine(4, 190, 120, 190, GREEN);
  tft.drawLine(4, 210, 120, 210, GREEN);
  tft.drawLine(4, 230, 120, 230, GREEN);
  tft.drawLine(24, 150, 24, 250, GREEN);
  tft.drawLine(44, 150, 44, 250, GREEN);
  tft.drawLine(64, 150, 64, 250, GREEN);
  tft.drawLine(84, 150, 84, 250, GREEN);
  tft.drawLine(104,150, 104, 250,GREEN);
 */
 tft.setTextColor(WHITE);
   tft.setTextSize(2);
   tft.setCursor(20,10);
   tft.println("   Portable EKG");
   tft.setTextColor(RED);
   //tft.setCursor(0,100);
   //tft.println("    Caution not a      medical device"); 
 
    
    V1= analogRead(5);
    tft.drawLine(5, 30, tft.width()-1, 30, RED);
    tft.drawLine(5, 130, tft.width()-1, 130, RED);
    tft.setTextColor(YELLOW,BLACK);
    
    tft.setTextSize(2);
    
    tft.setCursor(45, 40);
    tft.println(" PULSE ");
   
    tft.setCursor(40, 80);
    tft.setTextSize(3);
    tft.setTextColor(RED,BLACK );
    tft.print((((V1*VREF) / 1023)) * Dv1, 2);
    tft.setTextColor(YELLOW,BLACK);
    tft.print(" BPM ");
 while ( x1 < 235 ){ 
  
    y1[x1]= analogRead(5);
     Serial.println(x1);
     Serial.println(y1[x1]);
      y1[x1] = map(y1[x1], 0, 1023, 0, 120);
    tft.drawLine( x1+5, y1[x1]+180,x1+5,y1[x1 +1]+180,WHITE);
    x1 ++;
   delay(10);  
}
   
x1=0;
    
    }




