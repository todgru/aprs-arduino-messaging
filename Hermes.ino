/* 
 * Copyright 2012 Michael Pechner NE6RD
 * Distributed under the terms of the MIT License:
 * http://www.opensource.org/licenses/mit-license
 */

/*
Fkey Definitions - TBD
F1 - setup
F2 - noop
F3 - maually send beacon
F4-F6 - send stored message
F7 - toggle listen for
F8 - toggle Send to
F9-F12 - Future Use
*/

/*
Setup Menu  - TBD
1 Mycall
  view
  set
2 APRS ICON
  view - characters
  set characters
  --since we want to keep N messages, do not keep the table
 3. Path
   view
   set
   default "#default_path
 4. status message
    view set
 5. beacon interval
    view
    set upto nnn seconds
 6. Send_to
     view
     set
     clear
     on/off
 6. Listen For
    view list
    delete
    add
    clear all
    on/off
 7. GPS Speed
    view
    set
 8. ACC speed
    view
    set
 10. set messages
   view
   change
   clear
   clear all
*/

#define VERSION "1.1"
#include <stdlib.h>
#include <Wire.h>
#include <inttypes.h>
#include <EEPROM.h>
#include <NewSoftSerial.h>

#define ADAFRUIT_LCD 

#ifdef NEWHAVEN_LCD
// For the http://www.newhavendisplay.com I2C LCD display
#include <LCDi2cNHD.h>
LCDi2cNHD lcd = LCDi2cNHD(4,20,0x50>>1,0);
#endif

// For the Adafruit I2C/SPI Backpack and a 4x20 display
// http://www.ladyada.net/products/i2cspilcdbackpack/ 
// Install the modified LiquidCrystal library
// https://github.com/adafruit/LiquidCrystal
#ifdef ADAFRUIT_LCD
#include <LiquidCrystal.h>
// Connect via i2c, default address #0 (A0-A2 not jumpered)
LiquidCrystal lcd(0);
#endif

// Library from http://www.pjrc.com/teensy/td_libs_PS2Keyboard.html
// I added code to define more keys.
// If you get compilation errors about undefined PS2 symbols, 
// use the PS2Keyboard library from the book website.
#include <PS2Keyboard.h>
PS2Keyboard keyboard;

/*
EEPROM definitions
*/
//callsigns are 9 characters space padded
#define CALLS_SIZE 9
//Unless I misread the APRS spec, 63 characters uncluding the "{999" message counter.
// 59 = 63 - 4
#define MSG_SIZE 59
//The maximum number of callsigns that will be listened for if filtering incomming messages
#define NUM_LISTEN_FOR 5
//The number of characters for the GPS speed
#define SER_SPEED_LEN 5
//GPS Speed default
#define DEFAULT_GPS_SPEED '4800'
//the second serial port is for accessories - TDB
#define DEFAULT_ACC_SPEED 0
//A reasonable APRS digipath
#define DEFAULT_PATH "WIDE1-1,WIDE2-1"

//If you change the LCD, make sure you change the next few defines
//Number of lines in the LCD
#define LCD_LINES 4
//Characters on each line of the LCD
#define LCD_LINE_SZ 20
//Max characters the LCD can display
#define LCD_LEN 80

//The number of messages we will track
#define Q_SIZE 8
//The maximum length of any APRS message we will save
#define Q_LINE_LEN 80

//This is the structure we will use to store parameters between uses
struct eeprom_map_struct{
  //mycall
  char mycall [10];
  //If doing station to station, that call sign
  char msgto[10];
  //A flag '1' for station to station, '0' for a general status message
  byte msgto_flag;
  //Array to hold upto 5 callsigns we will filter for
  char listen_for[5][10];
  //A flag '1' to filter for listen_for callsigns,  '0' to receive messages from all stations
  byte listen_for_flag;
  //The digipath
  char path[25];
  //speed of the GPS - 0 means ignore
  //Since we are using softserial, keep the speed below 9600
  char gps_speed[5];
  //speed of the accessory - 0 means ignore
  //Since we are using softserial, keep the speed below 9600
  char acc_speed[5];
  //The aprs symbol to use.  C string, 2 characters and the null
  char aprs_sym[3];
  //The default status message for beacons
  char status_msg[MSG_SIZE+1];
  //How often status_msg is sent in seconds
  char beacon_every[4];
  //message send when F4, F5 or F6 is pressed
  char f4_msg[MSG_SIZE+1];
  char f5_msg[MSG_SIZE+1];
  char f6_msg[MSG_SIZE+1];
} eeprom_map;

//The message queue
char line_buffer[Q_SIZE][Q_LINE_LEN+1];
//Tracks which character we have read got the current message being received between -1 and Q_LINE_LEN
int q_char_cur =-1;
//The line we filling between 0 and Q_SIZE
int q_line_cur = -1;
//Which line in line_buffer is being displayed on the LCD
int d_line_cur = 0;
//To keep things from getting too jumpy we track the last time the up or down arrow is pressed.
// If the arrow key has not been pressed for more than 15 seconds, then when a new line is received
// q_line_cur and d_line_cur will be the same so the new received message will be displayed.
long last_arrow = millis();

//The lcd buffer
char lcdbuff[(LCD_LINES *  LCD_LINE_SZ)+1];


// 4 line LCDs do not display in order.  The lines are interleived. So to insert the second line:
// line[1] returns 2.  So the second line starts at character 2 * LCD_LINE_SZ or 40.
//First line starts at charaxter 0
//Second Line 40
//Third line 20
//Foruth line 60
//Warps the mind a bit doesn't it.
int line[4]={0,2,1,3};
/*
PIN Definitions
*/
//Radio shield d4 d5
/*
I am hoping to get softserial working with the radioshield.  For now it is really D0 & D1, the uart
*/
#define RADIO_SHIELD_TX 4
#define RADIO_SHIELD_RX 5

//LCD I2C A4 and A5
#define LCD_CLK 5
#define LCD_SDI 4

//GPS D6 D7
#define GPS_TX 6
#define GPS_RX 7

//Accessory Port  D8 D9
#define ACC_TX 8
#define ACC_RX 9

// PS/2 keyboard  D2 D3
const int DataPin = 2;
const int IRQpin =  3;

//                           rx tx
//NewSoftSerial  rs(RADIO_SHIELD_RX, RADIO_SHIELD_TX, true);


//Once we are using the eeprom, this will write the 
// Structure into the eeprom
//not used yet
void writeEeprom()
{ 
  char *ptr = (char *) eeprom_map.mycall;
  int s_size = sizeof(eeprom_map);
  int ii=0;
 while(ii <  s_size)
 {
   EEPROM.write(ii, *(ptr+ii));
   ii++;
 }
}

//
// Reads the eeprom
// Not used yet.
void readEeprom()
{
  
  char *ptr = (char *) eeprom_map.mycall;
  int s_size = sizeof(eeprom_map);
  int ii=0;
  int i255count = 0;
  //Read the eeprom and if the value is 255 or 0xff,track count of those bytes
 while(ii <  s_size)
 {
   *(ptr+ii) = EEPROM.read(ii);
   if ( *(ptr+ii) == 255 ) i255count++;
   ii++;
 }
 //IF we found 255 in at least 1/4 of the area, assume a new arduino and initialize it.
 if(i255count >= (s_size/4))
 {
   //initialize struct and eeprom
   memset(&eeprom_map,0, s_size);
   eeprom_map.msgto_flag='0';
   eeprom_map.listen_for_flag = '0';
   memcpy(eeprom_map.path, DEFAULT_PATH, strlen(DEFAULT_PATH));
   memcpy(eeprom_map.gps_speed, "0", 2);
   memcpy(eeprom_map.gps_speed, "0", 2);
   memcpy(eeprom_map.aprs_sym, "/D", 2);
   memcpy(eeprom_map.beacon_every, "0", 2);
   writeEeprom();
 }
 
}

/*
Will correct the value if it is outside the allowable values for line number
Used for up and down arrow key presses
*/
void check_line(int * line, int dir)
{
    int start = *line;
    if ( *line < 0)
    {   
        *line = Q_SIZE  - 1;
        while (line_buffer[*line][0] == '\0' && *line != start)
            *line = *line + dir;

    }
    if ( *line > Q_SIZE || line_buffer[*line][0] == '\0')
      *line=0;

}

//returns a valud next line
 void next_line(int * line)
{
        
    (*line)++;
    if ( *line >= Q_SIZE)
    {
        *line=0;
    }
 }
 
/*
Displays the text at line_buffer[d_line_cur]
*/
void display_lcd()
{
    
    memset(lcdbuff, ' ', LCD_LINES*LCD_LINE_SZ);
    lcdbuff[80]=0;
    
    int line_len = strnlen(line_buffer[d_line_cur], Q_LINE_LEN);
   
    //FILL THE BUFFER
    //Remember, because of the odd arrangement of the 4 lines, we can't just copy the line_buffer into the lcdbuff.
    //We need to grab each chunk of 20 and place it in the correct segment of lcdbuff.
    int ii = 0;
    int to_copy=0;
    int pos = 0;
    while(ii < 4 and pos < line_len)
    {
        to_copy = (line_len - pos) > 20 ? 20 : (line_len - pos);
        strncpy(&(lcdbuff[line[ii]*LCD_LINE_SZ]), &(line_buffer[d_line_cur][pos]), to_copy);
        pos += to_copy;
        ii++;
    }
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(lcdbuff);

}

/*
Send the line to the readio shield to transmit the APRS packet.
station to station: !:tocall : message {999
Beneral beacon  
*/
void sendline(char * message)
{
  int len = strnlen( message, Q_LINE_LEN);
  
  if ( len >= Q_LINE_LEN )
    *(message + Q_LINE_LEN) = '\0';
   
  if (eeprom_map.msgto_flag == '1')
  {
    Serial.print("W \r\n");// makes sure the i/o channel is cleared of cruft from any debug messages
    Serial.print("!:");
    Serial.print(eeprom_map.msgto);
    Serial.print(":");
  }
  else
  {
     Serial.print("!>");
  }
  Serial.print(message);
  Serial.print( "{");
  Serial.print(random(999));
  Serial.println("\r\n");
  Serial.println("Wsend message\r\n");
  delay(3000);
}

//Takes pos, a message offset and returns the position in the LCD buffer to write the character
//Again with the odd line offsets for a 4 line LCD
int which_pos(int pos)
{
   return ( ( line[ pos/LCD_LINE_SZ] * LCD_LINE_SZ) + (pos % LCD_LINE_SZ) );
}

//OK some people prefer DEFINE, I prefer functions.
int is_print(int c)
{
   return (c >= 32 && c <=128);
}

/*
Will read the data from the keyboard.  
If nothing is typed for 10 seconds, it is assumed the message is abandoned.

If you look, the first character read on loop() is passed in to be processed.

*/
void process_kb(int c)
{
     
  if (! is_print(c))
    return; 

   //line buffer for keybaord input
   char kb_line[Q_LINE_LEN + 1];
   int cur_char = 0;
   //timer var
   unsigned long st_millisec  = millis();
   //how long a pause in typing we accept
   const long max_milli = 10000L;
   
   //clear the LCD and  initial ize things
   lcd.clear();
   lcd.setCursor(0,0);
   memset(lcdbuff, ' ', LCD_LINES*LCD_LINE_SZ);
   memset(kb_line, '\0', Q_LINE_LEN + 1);
   lcdbuff[80]=0;
   //place the first character that was passed in.
   lcdbuff[0]= (char ) c;
   kb_line[0]=(char)c;
   
   int char_num = 1;
  
 
   while(1)
   {
     if ( keyboard.available() )
     {
       c = keyboard.read();
       //if return id some non printable key is pressed,
       // Send the message and update the LCD
       if (c == '\n' or ! is_print(c))
       {
         if( strlen(kb_line) > 0)
         {
           next_line(&q_line_cur);
           strncpy(line_buffer[q_line_cur], kb_line, Q_LINE_LEN);
           sendline(kb_line);
         }
         display_lcd();
         return;
       }
       st_millisec  = millis();//restart the 10sec timer
       //store the character
       lcdbuff[which_pos(char_num)] = (char)c;
       kb_line[char_num] = (char)c;
       
       char_num++;
       //update the LCD
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print(lcdbuff);
       
       //just keep overlaying the last character
       if (char_num >= MSG_SIZE)
       {
         char_num--;
       }
     }
     //if we stopped typeing for too long, exit
     if ( (millis() - st_millisec)  > max_milli)
     {
       display_lcd();
       return;
     }
   }
 display_lcd();  
}

//Read the messages from the radio shield
//
//We wil exit if we are reading the radio shield for longer 
// than 500ms or if something is being typed on the keyboard.
// 
//The radioshield is a 256 byte buffer
//
void process_radioshield()
{
    char inbyte = 0;  
    //max time to run this method
    const long max_milli = 500L; 
    unsigned long st_millisec  = millis();
    
    // -1 means the last time in we finished processing the last line
    if ( q_char_cur == -1 )
    {
        next_line(&q_line_cur); //get next offset in line_buffer to fill
        //If the user is scrolling through the messages, don't move d_line_cur/
        // If what is displayed for 15 seconds after the last time the up or down 
        //arrow key was pressed. set both cursors to the same spot.
        if (( millis() - last_arrow) > 15000L)
          d_line_cur = q_line_cur;
        //start at the first character  
        q_char_cur=0;
    }
        
  

    //while something is there and we are here less than mac_milli read from the readiosheild
    while (Serial.available() > 0  && (millis() - st_millisec)  < max_milli )
    {
        inbyte = Serial.read();      // Get the byte
        
        //end of line or message too long
        if ( inbyte == '\n' or q_char_cur >= Q_LINE_LEN)
        {
                       
            display_lcd();
            //if too many chaacters, read until end of line found
            if (q_char_cur >= Q_LINE_LEN){
                while(inbyte != '\n' and inbyte != '\r')
                    inbyte = Serial.read();
            }
            //set position for ack we processed a while message
            q_char_cur = -1;
            return;
        }
        else if (isprint(inbyte) and q_char_cur < Q_LINE_LEN)  // Only record printable characters
        {           
            //is printable and we have not read too much         
            line_buffer[q_line_cur][q_char_cur++] = inbyte;
            line_buffer[q_line_cur][q_char_cur]=0;                      
        }
        
        //kyboard data waiting leave
        if (keyboard.available())
        {
            return;
        }
    }     
    display_lcd();
}

//initialize everything
//If the setup does not seem to complete, make sure you have a keyboard installed

void setup()
{
#ifdef NEWHAVEN_LCD 
  lcd.init();
#endif
#ifdef ADAFRUIT_LCD
  lcd.begin(20, 4);
#endif
  lcd.setBacklight(5);
  lcd.clear();
  lcd.setCursor(0,0);
  
  Serial.begin(4800);
 
  //Makes sure that the line array is correct for a 2 line LCD.
  if ( LCD_LINES == 2)
        line[1]=1;

  memset(line_buffer, 0, sizeof(line_buffer));

  keyboard.begin(DataPin, IRQpin);
  //initialize the radom number generator - used in send_liine
  randomSeed(analogRead(0));

  //until I get the eeprom menu settings done, just hardcode them.
  /*
  NOTE: make sure mycall and msgto are space padded to 9 characters
  */
  memcpy(eeprom_map.mycall, "NE6RD-1  \0",10); 
  memcpy(eeprom_map.msgto,  "NE6RD-2  \0", 10);
  eeprom_map.msgto_flag='1';
  memcpy(eeprom_map.listen_for[0], "         \0", 10);
  eeprom_map.listen_for[0][0]='\0';
  eeprom_map.listen_for[1][0]='\0';
  eeprom_map.listen_for[2][0]='\0';
  eeprom_map.listen_for[3][0]='\0';
  eeprom_map.listen_for[4][0]='\0';
  eeprom_map.listen_for_flag = '1';
  strcpy(eeprom_map.path, DEFAULT_PATH);
  memcpy(eeprom_map.gps_speed, "0\0", 2);
  memcpy(eeprom_map.gps_speed, "0\0", 2);
  memcpy(eeprom_map.aprs_sym, "/D\0", 2);
  strcpy(eeprom_map.status_msg, " APRS TERMINAL \0");
  memcpy(eeprom_map.beacon_every, "0\0", 2);
  eeprom_map.f4_msg[0]='\0';
  eeprom_map.f5_msg[0]='\0';
  eeprom_map.f6_msg[0]='\0';
 
  //initialize the radio shield to it's callsign and the digipath
  Serial.print("M");
  Serial.print(eeprom_map.mycall);
  Serial.print("\r\n");
  Serial.read();//flush the return value from the input
  Serial.print("P");
  Serial.print(eeprom_map.path);
  Serial.print("\r\n");
  Serial.read();//flush the return value from the input
  delay(10);
  
  lcd.print("APRS TERMINAL");
  lcd.setCursor(1,0);
  lcd.print(" by NE6RD");
  lcd.setCursor(2,0);
  lcd.print(" Have Fun");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
}

// for ever
void loop()
{ 
  if (Serial.available() > 0 )
  {
    process_radioshield();
  }
  
  if ( keyboard.available())
  {
    int  c = keyboard.read();
    switch (c)
    {
        case PS2_DOWNARROW:
            d_line_cur--;
            check_line(&d_line_cur, +1);
            last_arrow=millis();
            display_lcd();
            c=0;
            break;
        case PS2_UPARROW:
            d_line_cur++;
            check_line(&d_line_cur, -1);
            display_lcd();
            last_arrow=millis();
            c=0;
            break;
    }
    //this is why we set 'c' tp 0 in the switch statement
    if (is_print(c))
    {
      process_kb(c);
    }

  }
}
