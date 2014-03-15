#include <XBee.h>
//#include "pitches.h"
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

/* LED alternate for testing P.O.V. and LEDs */
const int Mosfet[] = {22,19,20,21,18};    //actual connected column is Mosfet[0] pin 14 (connected to transistor)
const int oe = 2;
const int le = 3;
const int clk = 4;
const int sdi = 23;
int t = 1500;         // t=6667 is about 30Hz, so 1000us is 1/((0.001+0.000012+0.000032*6)*5)=166Hz
int zLED = 1;            //for delayMicroseconds to generate clock and accurately write SDI info
int rowDISPLAY = 0;
int faceDISPLAY = 0;
int columnDISPLAY = 0;
boolean LEDs[6][5][16];         //5 columns each have 6 faces each have 16 rows (only 15 LEDs though)
boolean redColumn[16];
boolean blueColumn[16];
boolean greenColumn[16];
boolean emptyColumn[16];
int location[3];
/* WIRELESS STUFF: */
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();// create reusable response objects for responses we expect to handle 
Rx64Response rx64 = Rx64Response();
boolean LEDreceived[6][5][16];
uint8_t payload[32];// allocate 32 bytes for to hold a 10-bit analog reading
Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload));// 16-bit addressing: Enter address of remote XBee, typically the coordinator
TxStatusResponse txStatus = TxStatusResponse();
const int XbeeSleep = 6;
uint8_t option = 0;
uint8_t data = 0;
boolean communication = 1;
int wirelessIndex = 0;
int receivedGameState = 0;
/* ACCELEROMETER STUFF: */
const int ZeroG = 13;
const int read_accel_x = 14;
const int read_accel_y = 15;
const int read_accel_z = 16;
const int read_averaging = 5000;/*determines how many readings the ADC averages before sending to cpu*/
const int read_smoothing = 10;/*determines how 'smooth' the readings are*/
const int wiggle = 75;/*determines wiggle room in determining what side is on top */
const int jerkwiggleneg = 50;/*determines the wiggle room for the jerk movements*/
const int jerkwigglepos = 50;
const int AVE_VALUE = 100;
int x[read_smoothing];
int y[read_smoothing];
int z[read_smoothing];
int accel_x_value;
int accel_y_value;
int accel_z_value;
int xval;
int yval;
int zval;
int zz = 0;
int x_average_value = 0;
int y_average_value = 0;
int z_average_value = 0;
int accel_points_taken = 0;
int x_hysteresis = 0;
int y_hysteresis = 0;
int z_hysteresis = 0;

/*GAME STUFF: */
int GameState = -1;/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A13));
/*ACCELEROMETER:*/  
  pinMode(ZeroG, INPUT);
  analogReadRes(10);
  analogReference(DEFAULT);
  analogReadAveraging(read_averaging);
  
/*WIRELESS:*/
  pinMode(XbeeSleep, OUTPUT);
  Serial3.begin(9600);
  xbee.setSerial(Serial3);      //set the xbee to rx3, pin 7 (to xbee pin 2),  and tx3, pin 8 (to xbee pin 3).
  digitalWrite(XbeeSleep, 1);	//0 wakes up the xbee, 1 puts it to sleep
  
/*LED:*/
  pinMode(Mosfet[0], OUTPUT);
  pinMode(Mosfet[1], OUTPUT);
  pinMode(Mosfet[2], OUTPUT);
  pinMode(Mosfet[3], OUTPUT);
  pinMode(Mosfet[4], OUTPUT);
  pinMode(oe, OUTPUT);
  pinMode(le, OUTPUT);
  pinMode(sdi, OUTPUT);
  pinMode(clk, OUTPUT);
  location[0] = 4;
  location[1] = 3;
  location[2] = 3;
  int i;
  columnDISPLAY = 0;
  while(columnDISPLAY<5){
    rowDISPLAY = 0;
    while(rowDISPLAY<16){      //initialize LEDs to be completely off
          faceDISPLAY = 0;
	  while(faceDISPLAY<6){
		  LEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
		  LEDreceived[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
		  faceDISPLAY++;
	  }
          rowDISPLAY++;
    }
    columnDISPLAY++;
  }
  
  i = 0;
  while(i<16){
    emptyColumn[i] = LOW;
    redColumn[i] = LOW;
    blueColumn[i] = LOW;
    greenColumn[i] = LOW;
    i++;
  }
  i = 1;
  while(i<16){
    redColumn[i] = HIGH;
    i=i+3;
  }
  i = 2;
  while(i<16){
    blueColumn[i] = HIGH;
    i=i+3;
  }
  i = 3;
  while(i<16){
    greenColumn[i] = HIGH;
    i=i+3;
  }
}

int pop(int *array){
  zz = read_smoothing - 1;
  while(zz > 0){
    array[zz] = array[(zz-1)];
    zz--;
  }
  return 0;
}

/*Calculates the change in a list with a pushed in value*/
/*Need to pass in the array you want to write to, and the read pin */
int accelChange(int *accellist, int newvalue){
  pop(accellist);
  accellist[0] = newvalue;
  return 0;
}

int updateaccel(int *x, int *y, int *z){
  accelChange(x, analogRead(read_accel_x));
  accelChange(y, analogRead(read_accel_y));
  accelChange(z, analogRead(read_accel_z));
  return 0;
}

void updateaccelOTHER(){ //adds the reading from the accelerometer to the previous readings
	x_average_value = x_average_value + analogRead(read_accel_x);
	y_average_value = y_average_value + analogRead(read_accel_y);
	z_average_value = z_average_value + analogRead(read_accel_z);
	accel_points_taken++;
}

void accelAveWithHysteresis(){ //rounds the x,y,z values to the nearest ten based on previous value
	if(x_hysteresis==0){	//for first time use: just round normally
		if((x_average_value%10)<5){	//round down
			x_hysteresis = x_average_value - (x_average_value%10);
			y_hysteresis = y_average_value - (y_average_value%10);
			z_hysteresis = z_average_value - (z_average_value%10);
		}else{		//round up
			x_hysteresis = x_average_value + (10-(x_average_value%10));
			y_hysteresis = y_average_value + (10-(y_average_value%10));
			z_hysteresis = z_average_value + (10-(z_average_value%10));
		}
	}else{		//keeps it to previous value unless it's a significant amount higher or lower
		if(x_average_value > x_hysteresis){  // X HYSTERESIS VALUE
			if((x_average_value - x_hysteresis)>7){ //significantly higher
				x_hysteresis = x_hysteresis + 10;
			}
		}else{ //smaller
			if((x_hysteresis - x_average_value)>7){ //significantly lower
				x_hysteresis = x_hysteresis - 10;
			}
		}
		if(y_average_value > y_hysteresis){  // Y HYSTERESIS VALUE
			if((y_average_value - y_hysteresis)>7){ //significantly higher
				y_hysteresis = y_hysteresis + 10;
			}
		}else{ //smaller
			if((y_hysteresis - y_average_value)>7){ //significantly lower
				y_hysteresis = y_hysteresis - 10;
			}
		}
		if(z_average_value > z_hysteresis){  // Z HYSTERESIS VALUE
			if((z_average_value - z_hysteresis)>7){ //significantly higher
				z_hysteresis = z_hysteresis + 10;
			}
		}else{ //smaller
			if((z_hysteresis - z_average_value)>7){ //significantly lower
				z_hysteresis = z_hysteresis - 10;
			}
		} //if not significant change, don't change the hysteresis value
			//NOTE: this assumes the values don't change by more than 18 at a time. Fix?
	}		
	Serial.print("X: ");
	Serial.print(x_hysteresis);
	Serial.print(" Y: ");
	Serial.print(y_hysteresis);
	Serial.print(" Z: ");
	Serial.println(z_hysteresis);
}

void accelaveOTHER(){ //takes the sum of the readings and divides it by how many readings there were to get the average
	x_average_value = x_average_value/accel_points_taken;
	y_average_value = y_average_value/accel_points_taken;
	z_average_value = z_average_value/accel_points_taken;
	accel_points_taken = 0;
	accelAveWithHysteresis();
}

void clearaccelOTHER(){
	x_average_value = 0;
	y_average_value = 0;
	z_average_value = 0;
}

int accelave(int *array){
  int i = 0;
  zz = 0;
  for (i = 0; i < read_smoothing; i = i + 1) {
      zz += array[i];
  }
  return zz/read_smoothing;
}

int acceltop(int *x, int *y, int *z){
  /*xval = accelave(x);
  yval = accelave(y);
  zval = accelave(z);*/
  xval = analogRead(read_accel_x);
  yval = analogRead(read_accel_y);
  zval = analogRead(read_accel_z);
  
  if(xval > (256 - wiggle) && xval < (256 + wiggle) && yval > (558 - wiggle) && yval < (558 + wiggle) && zval > (413 - wiggle) && zval < (413 + wiggle)){
    return 0;//f
  }
  else if(xval > (519 - wiggle) && xval < (519 + wiggle) && yval > (740 - wiggle) && yval < (740 + wiggle) && zval > (558 - wiggle) && zval < (558 + wiggle)){
    return 4;//f
  }
  else if(xval > (532 - wiggle) && xval < (532 + wiggle) && yval > (648 - wiggle) && yval < (648 + wiggle) && zval > (212 - wiggle) && zval < (212 + wiggle)){
    return 1;//f
  }
  else if(xval > (482 - wiggle) && xval < (482 + wiggle) && yval > (403 - wiggle) && yval < (403 + wiggle) && zval > (629 - wiggle) && zval < (629 + wiggle)){
    return 3;//f
  }
  else if(xval > (493 - wiggle) && xval < (493 + wiggle) && yval > (324 - wiggle) && yval < (324 + wiggle) && zval > (290 - wiggle) && zval < (290 + wiggle)){
    return 2;//f
  }
  else if(xval > (740 - wiggle) && xval < (740 + wiggle) && yval > (510 - wiggle) && yval < (510 + wiggle) && zval > (453 - wiggle) && zval < (453 + wiggle)){
    return 5;//f
  }
  else{
    return 6;
  }
}

int acceljerk(int *x, int *y, int *z) {
  /*xval = accelave(x);
  yval = accelave(y);
  zval = accelave(z);*/
  xval = analogRead(read_accel_x);
  yval = analogRead(read_accel_y);
  zval = analogRead(read_accel_z);
  if(xval < (0 + jerkwiggleneg) || xval > (1023 - jerkwigglepos) || yval < (0 + jerkwiggleneg) || yval > (1023 - jerkwigglepos) || zval < (0 + jerkwiggleneg) || zval > (1023 - jerkwigglepos)){
    return 1;
  }
  else {
    return 0;
  }
}

void displayLEDs(boolean LED[6][5][16]){

  digitalWrite(clk,0);
  digitalWrite(sdi,LOW);
    
  columnDISPLAY=0;
  while(columnDISPLAY<5){    //for displaying each column
     digitalWriteFast(clk,1);         //Prime the clock                    
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     delayMicroseconds(zLED);
     digitalWriteFast(clk,1);                            
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     digitalWriteFast(clk,1);                             
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     delayMicroseconds(zLED);
     digitalWriteFast(clk,1);                            
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     digitalWriteFast(le,HIGH);         //Begin
     delayMicroseconds(zLED);
     digitalWriteFast(clk,1);                            
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     digitalWriteFast(le,LOW);
     digitalWriteFast(oe,HIGH); 
     delayMicroseconds(zLED);
     digitalWriteFast(clk,1);     
     
     faceDISPLAY = 0; 
     while(faceDISPLAY<6){   //iterate through to write the correct row information on one column for all 6 faces
       rowDISPLAY = 0;
       while(rowDISPLAY<16){
         delayMicroseconds(zLED);
         digitalWriteFast(clk,0);
         digitalWriteFast(sdi, LED[faceDISPLAY][columnDISPLAY][rowDISPLAY]);       //Send SDI Info
         delayMicroseconds(zLED);     
         digitalWriteFast(clk,1);                            
         
         rowDISPLAY++;
       }
       rowDISPLAY = 0;
       faceDISPLAY++;
     }   
  
     digitalWriteFast(le,HIGH);         //End  
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     digitalWriteFast(le,LOW);
     digitalWriteFast(sdi, LOW);      
     delayMicroseconds(zLED);
     digitalWriteFast(clk,1); 
     delayMicroseconds(zLED);
     digitalWriteFast(clk,0);
     digitalWriteFast(oe,LOW);
     
     digitalWrite(Mosfet[columnDISPLAY],0);       //turn the correct column on
     delayMicroseconds(t);        //keep the column on for a little while
     digitalWrite(Mosfet[columnDISPLAY], 1);  //then turn the mosfet off so we can move to the next column
     updateaccelOTHER();     								/*KENT YOU CAN CHANGE THIS TO PUSH AND POP*/
	 delayMicroseconds(10);
     updateaccelOTHER();     								/*KENT YOU CAN CHANGE THIS TO PUSH AND POP*/
	 columnDISPLAY++;                         //move to the next column
  }
}

void TurnOnSingleLED(int face, int column, int row, int color){//red=0,green=1,blue=2, rg=3, rb=4, gb = 5, all=6
  int row1 = (row)*3;
  int row2 = (row)*3-1;
  int row3 = (row)*3-2;
  boolean combos1 = LOW;
  boolean combos2 = LOW;
  boolean combos3 = LOW;
  if(color == 3){
    color = 0;
    combos2 = HIGH;
  }else if(color == 4){
    color = 0;
    combos3 = HIGH;
  }else if(color == 5){
    color = 1;
    combos3 = HIGH;
  }else if(color == 6){
    color = 0;
    combos2 = HIGH;
    combos3 = HIGH;
  }
  row = (row)*3-color;
  LEDs[face][column-1][row1] = combos1;
  LEDs[face][column-1][row2] = combos2;
  LEDs[face][column-1][row3] = combos3;
  LEDs[face][column-1][row] = HIGH;
}

void TurnOffSingleLED(int face, int column, int row){
	row = (row+1)*3;
	LEDs[face][column][row] = LOW;
	LEDs[face][column][row-1] = LOW;
	LEDs[face][column][row-2] = LOW;
}

void TurnOnRow(int face, int row, int color){//red=0,green=1,blue=2
  int row1 = (row+1)*3;
  int row2 = (row+1)*3-1;
  int row3 = (row+1)*3-2;
  row = (row+1)*3-color;
  int column = 0;
  while(column<5){
    LEDs[face][column][row1] = LOW;
    LEDs[face][column][row2] = LOW;
    LEDs[face][column][row3] = LOW;
    LEDs[face][column][row] = HIGH;
    column++;
  }
}

void TurnOffRow(int face, int row){
  int row1 = (row+1)*3;
  int row2 = (row+1)*3-1;
  int row3 = (row+1)*3-2;
  int column = 0;
  while(column<5){
    LEDs[face][column][row1] = LOW;
    LEDs[face][column][row2] = LOW;
    LEDs[face][column][row3] = LOW;
    column++;
  }
}

void TurnOnColumn(int face, int column, int color){//red=0,green=1,blue=2
  int row = 0;
  while(row<16){
    LEDs[face][column][row] = LOW;
    row++;
  }
  row = 3-color;
  LEDs[face][column][row] = HIGH;
  row = row+3;
  LEDs[face][column][row] = HIGH;
  row = row+3;
  LEDs[face][column][row] = HIGH;
  row = row+3;
  LEDs[face][column][row] = HIGH;
  row = row+3;
  LEDs[face][column][row] = HIGH;
}

void TurnOffColumn(int face, int column){
	int row = 0;
	while(row<16){
		LEDs[face][column][row] = LOW;
		row++;
	}
}

void TurnOnFace(int face, int color){//red=0,green=1,blue=2, rg=3, rb=4, gb = 5, all=6
	int icolumn = 1;
	int irow = 1;
	while(icolumn<6){
		irow = 1;
		while(irow<6){
			TurnOnSingleLED(face,icolumn,irow,color);
			irow++;
		}
		icolumn++;
	}
}

void TurnOnCubeLED(int color){//red=0,green=1,blue=2, rg=3, rb=4, gb = 5, all=6
	int iface = 0;
	int icolumn = 1;
	int irow = 1;
	while(iface<6){
		icolumn = 1;
		while(icolumn<6){
			irow = 1;
			while(irow<6){
				TurnOnSingleLED(iface,icolumn,irow,color);
				irow++;
			}
			icolumn++;
		}
		iface++;
	}
}

void TurnOffCubeLED(){
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //turn all LEDs off
			faceDISPLAY = 0;
			while(faceDISPLAY<6){
				LEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
				faceDISPLAY++;
			}
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
}

int CountShowOff = 0;
void ShowOff(){
  CountShowOff++;
  int row = 1;
  int column = 1;
  int color = 0;
  if(CountShowOff<10){
    while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  }else if(CountShowOff>9 && CountShowOff<20){
    while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row+1)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  }else if(CountShowOff>19 && CountShowOff<30){
    while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row+2)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  }else if(CountShowOff>29 && CountShowOff<40){
    while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row+3)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  }else if(CountShowOff>39 && CountShowOff<50){
    while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row+4)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  }else if(CountShowOff>49 && CountShowOff<60){
      while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row+5)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  }else if(CountShowOff>59){
    CountShowOff = 0;
  }
   
}

int CountShowOffAccel = 0;
void ShowOffAccel(){
  CountShowOffAccel++;
  int row = 1;
  int column = 1;
  int color = 0;
  //int AccelOffSet = (analogRead(read_accel_x) + analogRead(read_accel_y))/100;
  accelaveOTHER();
  int AccelOffSet = (x_hysteresis)/50;
  //Serial.println(x_average_value);
  clearaccelOTHER();
  //Serial.println(AccelOffSet);
  // if(CountShowOffAccel<10){
    while(column<6){
      row = 1;
      while(row<6){
        color = ((column+row+AccelOffSet)%6);
        TurnOnSingleLED(5,column,row,color);
        TurnOnSingleLED(4,column,row,color);
        TurnOnSingleLED(3,column,row,color);
        TurnOnSingleLED(2,column,row,color);
        TurnOnSingleLED(1,column,row,color);
        TurnOnSingleLED(0,column,row,color);
        row++;
      }
      column++;
    }
  // }else if(CountShowOffAccel>9 && CountShowOffAccel<20){
    // while(column<6){
      // row = 1;
      // while(row<6){
        // color = ((column+row+AccelOffSet+1)%6);
        // TurnOnSingleLED(5,column,row,color);
        // TurnOnSingleLED(4,column,row,color);
        // TurnOnSingleLED(3,column,row,color);
        // TurnOnSingleLED(2,column,row,color);
        // TurnOnSingleLED(1,column,row,color);
        // TurnOnSingleLED(0,column,row,color);
        // row++;
      // }
      // column++;
    // }
  // }else if(CountShowOffAccel>19 && CountShowOffAccel<30){
    // while(column<6){
      // row = 1;
      // while(row<6){
        // color = ((column+row+AccelOffSet+2)%6);
        // TurnOnSingleLED(5,column,row,color);
        // TurnOnSingleLED(4,column,row,color);
        // TurnOnSingleLED(3,column,row,color);
        // TurnOnSingleLED(2,column,row,color);
        // TurnOnSingleLED(1,column,row,color);
        // TurnOnSingleLED(0,column,row,color);
        // row++;
      // }
      // column++;
    // }
  // }else if(CountShowOffAccel>29 && CountShowOffAccel<40){
    // while(column<6){
      // row = 1;
      // while(row<6){
        // color = ((column+row+AccelOffSet+3)%6);
        // TurnOnSingleLED(5,column,row,color);
        // TurnOnSingleLED(4,column,row,color);
        // TurnOnSingleLED(3,column,row,color);
        // TurnOnSingleLED(2,column,row,color);
        // TurnOnSingleLED(1,column,row,color);
        // TurnOnSingleLED(0,column,row,color);
        // row++;
      // }
      // column++;
    // }
  // }else if(CountShowOffAccel>39 && CountShowOffAccel<50){
    // while(column<6){
      // row = 1;
      // while(row<6){
        // color = ((column+row+AccelOffSet+4)%6);
        // TurnOnSingleLED(5,column,row,color);
        // TurnOnSingleLED(4,column,row,color);
        // TurnOnSingleLED(3,column,row,color);
        // TurnOnSingleLED(2,column,row,color);
        // TurnOnSingleLED(1,column,row,color);
        // TurnOnSingleLED(0,column,row,color);
        // row++;
      // }
      // column++;
    // }
  // }else if(CountShowOffAccel>49 && CountShowOffAccel<60){
      // while(column<6){
      // row = 1;
      // while(row<6){
        // color = ((column+row+AccelOffSet+5)%6);
        // TurnOnSingleLED(5,column,row,color);
        // TurnOnSingleLED(4,column,row,color);
        // TurnOnSingleLED(3,column,row,color);
        // TurnOnSingleLED(2,column,row,color);
        // TurnOnSingleLED(1,column,row,color);
        // TurnOnSingleLED(0,column,row,color);
        // row++;
      // }
      // column++;
    // }
  // }else if(CountShowOffAccel>59){
    // CountShowOffAccel = 0;
  // }
   
}

int CountRandoCube = 0;
void RandoCube(){
	int i = 0;
	int iface = 0;
	int icolumn = 1;
	int irow = 1;
	if(CountRandoCube == 0){
		while(iface<6){
			icolumn = 1;
			while(icolumn<6){
				irow = 1;
				while(irow<6){
					TurnOnSingleLED(iface,icolumn,irow,(int) random(6));
					irow++;
				}
				icolumn++;
			}
			iface++;
		}
		CountRandoCube++;
	}
	i = (int) (15);
	while(i>0){
		TurnOnSingleLED((int) random(0,6),(int) random(1,6),(int) random(1,6),(int) random(0,6));
		i--;
	}
	i = (int) random(5);
	while(i>0){
		displayLEDs(LEDs);
		i--;
	}
}

int CountRandoFace = 0;
void RandoFace(int face){
	int icolumn = 1;
	int irow = 1;
	if(CountRandoFace%15 == 0){
		while(icolumn<6){
			irow = 1;
			while(irow<6){
				TurnOnSingleLED(face,icolumn,irow,(int) random(6));
				irow++;
			}
			icolumn++;
		}
	}
	CountRandoFace++;
}

void WhiteColor(){
	t = 500;
	int icolumn = 1;
	int irow = 1;
	while(icolumn<6){
		irow = 1;
		while(irow<6){
			TurnOnSingleLED(3,icolumn,irow,0);
			irow++;
		}
		icolumn++;
	}
	displayLEDs(LEDs);
	icolumn = 1;
	while(icolumn<6){
		irow = 1;
		while(irow<6){
			TurnOnSingleLED(3,icolumn,irow,1);
			irow++;
		}
		icolumn++;
	}
	displayLEDs(LEDs);
	icolumn = 1;
	while(icolumn<6){
		irow = 1;
		while(irow<6){
			TurnOnSingleLED(3,icolumn,irow,2);
			irow++;
		}
		icolumn++;
	}
	displayLEDs(LEDs);
	t = 1500;
}

void MoveLED(int face, int column, int row, int movePitch, int moveYaw, int moveRoll){ 
	int color=6;
	int tempRow = row*3;
	if(LEDs[face][column-1][tempRow]){
		color = 0;
	}else if(LEDs[face][column-1][tempRow-1]){
		color = 1;
	}else if(LEDs[face][column-1][tempRow-2]){
		color = 2;
	}
	if(LEDs[face][column-1][tempRow] && LEDs[face][column-1][tempRow-1]){
		color = 3;
	}else if(LEDs[face][column-1][tempRow] && LEDs[face][column-1][tempRow-2]){
		color = 4;
	}else if(LEDs[face][column-1][tempRow-1] && LEDs[face][column-1][tempRow-2]){
		color = 5;
	}else if(LEDs[face][column-1][tempRow] && LEDs[face][column-1][tempRow-1] && LEDs[face][column-1][tempRow-2]){
		color = 6;
	}
	if(face == 5){ //+column is +Roll, +row is +Pitch
		TurnOffSingleLED(face, column, row);
		if(column+moveRoll>5){
			face = 1;
			moveRoll = (column+moveRoll)-6;
			column = -(row-6);
			row = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch>5){
			face = 2;
			movePitch = (row-movePitch)-6;
			row = -(column-6);
			column = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		if(column+moveRoll<1){
			face = 3;
			moveRoll = column+moveRoll;
			column = row;
			row = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch<1){
			face = 4;
			movePitch = row-movePitch;
			row = column;
			column = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+moveRoll, row+movePitch, color);
                location[0] = face;
                location[1] = column+moveRoll;
                location[2] = row+movePitch;                
	}else if(face == 4){ //+column is -Pitch, +row is +Yaw
		TurnOffSingleLED(face, column, row);
		if(column-movePitch>5){
			face = 0;
			movePitch = (column-movePitch)-6;
			column = -(row-6);
			row = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row+moveYaw>5){
			face = 1;
			moveYaw = (row+moveYaw)-6;
			row = -(column-6);
			column = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		if(column-movePitch<1){
			face = 5;
			movePitch = column-movePitch;
			column = row;
			row = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row+moveYaw<1){
			face = 3;
			moveYaw = row+moveYaw;
			row = column;
			column = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column-movePitch, row+moveYaw, color);
                location[0] = face;
                location[1] = column-movePitch;
                location[2] = row+moveYaw;       
	}else if(face == 3){ //+column is -Yaw, +row is -Roll
		TurnOffSingleLED(face, column, row);
		if(column-moveYaw>5){
			face = 2;
			moveYaw = (column-moveYaw)-6;
			column = row;
			row = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll>5){
			face = 0;
			moveRoll = (row-moveRoll)-6;
			row = column;
			column = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		if(column-moveYaw<1){
			face = 4;
			moveYaw = column-moveYaw;
			column = row;
			row = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll<1){
			face = 5;
			moveRoll = row-moveRoll;
			row = column;
			column = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column-moveYaw, row-moveRoll, color);
                location[0] = face;
                location[1] = column-moveYaw;
                location[2] = row-moveRoll;       
	}else if(face == 2){ //+column is +Pitch, +row is +Yaw
		TurnOffSingleLED(face, column, row);
		if(column+movePitch>5){
			face = 0;
			movePitch = (column+movePitch)-6;
			column = row;
			row = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row+moveYaw>5){
			face = 3;
			moveYaw = (row+moveYaw)-6;
			row = column;
			column = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		if(column+movePitch<1){
			face = 5;
			movePitch = column+movePitch;
			column = -(row-6);
			row = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row+moveYaw<1){
			face = 1;
			moveYaw = row+moveYaw;
			row = -(column-6);
			column = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+movePitch, row+moveYaw, color);
                location[0] = face;
                location[1] = column+movePitch;
                location[2] = row+moveYaw;       
	}else if(face == 1){ //+column is +Yaw, +row is -Roll
		TurnOffSingleLED(face, column, row);
		if(column+moveYaw>5){
			face = 2;
			moveYaw = (column+moveYaw)-6;
			column = -(row-6);
			row = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll>5){
			face = 5;
			moveRoll = (row-moveRoll)-6;
			row = column;
			column = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		if(column+moveYaw<1){
			face = 4;
			moveYaw = column+moveYaw;
			column = -(row-6);
			row = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll<1){
			face = 0;
			moveRoll = row-moveRoll;
			row = column;
			column = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+moveYaw, row-moveRoll, color);
                location[0] = face;
                location[1] = column+moveYaw;
                location[2] = row-moveRoll;       
	}else if(face == 0){ //+column is +Roll, +row is -Pitch
		TurnOffSingleLED(face, column, row);
		if(column+moveRoll>5){
			face = 3;
			moveRoll = (column+moveRoll)-6;
			column = row;
			row = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch>5){
			face = 2;
			movePitch = (row-movePitch)-6;
			row = column;
			column = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		if(column+moveRoll<1){
			face = 1;
			moveRoll = column+moveRoll;
			column = row;
			row = 1;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch<1){
			face = 4;
			movePitch = row-movePitch;
			row = -(column-6);
			column = 5;
			MoveLED(face, column, row, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+moveRoll, row-movePitch, color);
                location[0] = face;
                location[1] = column+moveRoll;
                location[2] = row-movePitch;       
	}
} 

void translateReceivedLED(int rowReceived, int colorReceived, int indexReceived){
  int faceReceived = indexReceived/5; //index 0-4 is face 0, index 25-29 is face 5
  int columnReceived = indexReceived%5; //index 0 is column 0, index 4 is column 4, index 5 is column 0 again
  boolean row1 = (boolean) rowReceived&1; 
  boolean row2 = (boolean) (rowReceived&2) >> 1;
  boolean row3 = (boolean) (rowReceived&4) >> 2;
  boolean row4 = (boolean) (rowReceived&8) >> 3;
  boolean row5 = (boolean) rowReceived >> 4;    //10000
  LEDreceived[faceReceived][columnReceived][3-colorReceived] = row1;
  LEDreceived[faceReceived][columnReceived][6-colorReceived] = row2;
  LEDreceived[faceReceived][columnReceived][9-colorReceived] = row3;
  LEDreceived[faceReceived][columnReceived][12-colorReceived] = row4;
  LEDreceived[faceReceived][columnReceived][15-colorReceived] = row5;
  //Serial.print(faceReceived); //for rowSent = 26 = 0b11010, row1 = 0  (original sent character is 0b01011010 = 90
  //Serial.println(columnReceived);
  //Serial.println(colorReceived); 
}

void CopyLEDDisplayIntoPayload(int colorSend, int IDSend){  //fills payload for sending the color cube you specified
	int i = 0;
	while(i<30){
		payload[i] = (uint8_t) 64 + LEDs[i/5][i%5][3-colorSend] + LEDs[i/5][i%5][6-colorSend]*2 + LEDs[i/5][i%5][9-colorSend]*4 + LEDs[i/5][i%5][12-colorSend]*8 + LEDs[i/5][i%5][15-colorSend]*16;
                i++;
	}
	payload[30] = (uint8_t) 64 + colorSend;
	payload[31] = (uint8_t) IDSend;
}

void displayReceivedLEDs(boolean LED[6][5][16]){
	faceDISPLAY = 0;
	while(faceDISPLAY<6){
		columnDISPLAY = 0;
		while(columnDISPLAY<5){      //initialize LEDs to be completely off
			rowDISPLAY = 0;
			while(rowDISPLAY<16){
				LEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY] = LED[faceDISPLAY][columnDISPLAY][rowDISPLAY];
					if(LED[faceDISPLAY][columnDISPLAY][rowDISPLAY]){
						Serial.println("HIGH");
					}
				rowDISPLAY++;
			}
			columnDISPLAY++;
		}
		faceDISPLAY++;
	}
}

void StartUp(){
  int xLED;
  columnDISPLAY = 0;
  while(columnDISPLAY<5){
   rowDISPLAY = 0;
   while(rowDISPLAY<16){      //for turning one LED on at a time then after displaying for a while, switch to next LED
      if(rowDISPLAY%3==1){ //turn on green LEDs
        xLED=1;
      }else{
        xLED=0;
      }
      LEDs[5][columnDISPLAY][rowDISPLAY]=xLED;
      LEDs[4][columnDISPLAY][rowDISPLAY]=xLED;
      LEDs[3][columnDISPLAY][rowDISPLAY]=xLED;
      LEDs[2][columnDISPLAY][rowDISPLAY]=xLED;
      LEDs[1][columnDISPLAY][rowDISPLAY]=xLED;
      LEDs[0][columnDISPLAY][rowDISPLAY]=xLED;
      rowDISPLAY++;
   }
   columnDISPLAY++;
  }
  //SOUND CODE
  //import from Conrad
}

void SelectGame(){ //used at the after starting up for starting a game
	//updateaccel(x,y,z);
	//StartUp();
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //turn all LEDs off
			faceDISPLAY = 0;
			while(faceDISPLAY<6){
				LEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
				faceDISPLAY++;
			}
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
	TurnOnSingleLED(0,3,1,2);//top face show #1
	TurnOnSingleLED(0,3,2,2);
	TurnOnSingleLED(0,3,3,2);
	TurnOnSingleLED(0,3,4,2);
	TurnOnSingleLED(0,3,5,2);
	TurnOnSingleLED(4,2,1,2);//front face show #2
	TurnOnSingleLED(4,3,1,2);
	TurnOnSingleLED(4,4,1,2);
	TurnOnSingleLED(4,4,2,2);
	TurnOnSingleLED(4,4,3,2);
	TurnOnSingleLED(4,3,3,2);
	TurnOnSingleLED(4,2,3,2);
	TurnOnSingleLED(4,2,4,2);
	TurnOnSingleLED(4,2,5,2);
	TurnOnSingleLED(4,3,5,2);
	TurnOnSingleLED(4,4,5,2);
	TurnOnSingleLED(3,2,1,2);//front face show #3
	TurnOnSingleLED(3,3,1,2);
	TurnOnSingleLED(3,4,1,2);
	TurnOnSingleLED(3,4,2,2);
	TurnOnSingleLED(3,4,3,2);
	TurnOnSingleLED(3,3,3,2);
	TurnOnSingleLED(3,2,3,2);
	TurnOnSingleLED(3,4,4,2);
	TurnOnSingleLED(3,4,5,2);
	TurnOnSingleLED(3,3,5,2);
	TurnOnSingleLED(3,2,5,2);
	int GameSelected = 6;
	int i = 0;
	int Testaccel;
	int GameSel[] = {0,0,0};
	//while((GameSelected!=0&&GameSelected!=4&&GameSelected!=3)){
		//Serial.println(GameSelected);
		//updateaccel(x,y,z);
		Testaccel = acceljerk(x,y,z);
		//Serial.println(Testaccel);
		while(Testaccel==0){
			//updateaccel(x,y,z);
			Testaccel = acceljerk(x,y,z);
			displayLEDs(LEDs);
		}
		
		while(i<100){
			if(acceltop(x,y,z) == 0){
				GameSel[0] = GameSel[0]++;
			}
			if(acceltop(x,y,z) == 4){
				GameSel[1] = GameSel[1]++;
			}
			if(acceltop(x,y,z) == 3){
				GameSel[2] = GameSel[2]++;
			}
			i++;
			displayLEDs(LEDs);
		}
		if(GameSel[0]>GameSel[1] && GameSel[0]>GameSel[2]){
			GameState = 10;
		}else if(GameSel[1]>GameSel[0] && GameSel[1]>GameSel[2]){
			GameState = 20;
		}else{
			GameState = 30;
		}
		displayLEDs(LEDs);
		Serial.println(GameState);
	
}

void StartGame1(){ //navigate your blue guy across the cube, don't hit the rails, eat all the apples
        int melody[] = {
          NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B4, NOTE_A4, NOTE_G3, NOTE_F3,NOTE_E3,NOTE_D3,NOTE_C3, NOTE_B3, NOTE_A3, NOTE_G2,NOTE_F2,NOTE_E2};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {
          8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8, };	
        int i = 0;
        columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //turn all LEDs off
			faceDISPLAY = 0;
			while(faceDISPLAY<6){
				LEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
				faceDISPLAY++;
			}
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
	TurnOnSingleLED(0,1,1,0);//turn on red rails on top face
	TurnOnSingleLED(0,2,1,0);
	TurnOnSingleLED(0,3,1,0);
	TurnOnSingleLED(0,4,1,0);
	TurnOnSingleLED(0,5,1,0);
	TurnOnSingleLED(0,1,5,0);
	TurnOnSingleLED(0,2,5,0);
	TurnOnSingleLED(0,3,5,0);
	TurnOnSingleLED(0,4,5,0);
	TurnOnSingleLED(0,5,5,0);
	TurnOnSingleLED(4,1,1,0);//red rails on front face
	TurnOnSingleLED(4,2,1,0);
	TurnOnSingleLED(4,3,1,0);
	TurnOnSingleLED(4,4,1,0);
	TurnOnSingleLED(4,5,1,0);
	TurnOnSingleLED(4,1,5,0);
	TurnOnSingleLED(4,2,5,0);
	TurnOnSingleLED(4,3,5,0);
	TurnOnSingleLED(4,4,5,0);
	TurnOnSingleLED(4,5,5,0);
	TurnOnSingleLED(5,1,1,0);//red rails on bottom face
	TurnOnSingleLED(5,1,2,0);
	TurnOnSingleLED(5,1,3,0);
	TurnOnSingleLED(5,1,4,0);
	TurnOnSingleLED(5,1,5,0);
	TurnOnSingleLED(5,5,1,0);
	TurnOnSingleLED(5,5,2,0);
	TurnOnSingleLED(5,5,3,0);
	TurnOnSingleLED(5,5,4,0);
	TurnOnSingleLED(5,5,5,0);
	TurnOnSingleLED(2,1,1,0);//red rails on back face
	TurnOnSingleLED(2,2,1,0);
	TurnOnSingleLED(2,3,1,0);
	TurnOnSingleLED(2,4,1,0);
	TurnOnSingleLED(2,5,1,0);
	TurnOnSingleLED(2,1,5,0);
	TurnOnSingleLED(2,2,5,0);
	TurnOnSingleLED(2,3,5,0);
	TurnOnSingleLED(2,4,5,0);
	TurnOnSingleLED(2,5,5,0);
	TurnOnSingleLED(4,3,3,1);//apple in the middle of front face
	TurnOnSingleLED(5,3,3,1);//apple in middle of bottom face
	TurnOnSingleLED(2,3,3,1);//apple in middle of back face
	TurnOnFace(1,0);
	TurnOnFace(3,0);
	TurnOnSingleLED(0,3,3,2);//Your starting spot, you're blue
        for (int thisNote = 0; thisNote < 3; thisNote++) {
       
          // to calculate the note duration, take one second 
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1000/noteDurations[thisNote];
          tone(5, melody[thisNote],noteDuration);
      
          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.30;
          i=0;
          while(i<(pauseBetweenNotes/7)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
            i++;
          }
          noTone(5);
        }
}

void DisplayGame2(){
	updateaccel(x,y,z);
        int i = 0;
        int melody[] = {
          NOTE_F4,NOTE_D4, NOTE_C5};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {
          8, 8, 8};
	int xLED = 0;
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //for turning one LED on at a time then after displaying for a while, switch to next LED
			if(rowDISPLAY%(3*(columnDISPLAY%2+1))==1){ //0=red, 1=green, 2=blue
				xLED=1;
			}else{
				xLED=0;
			}
			LEDs[5][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[4][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[3][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[2][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[1][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[0][columnDISPLAY][rowDISPLAY]=xLED;
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
	displayLEDs(LEDs);
        for (int thisNote = 0; thisNote < 3; thisNote++) {
       
          // to calculate the note duration, take one second 
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1000/noteDurations[thisNote];
          tone(5, melody[thisNote],noteDuration);
      
          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.30;
          i=0;
          while(i<(pauseBetweenNotes/7)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
            i++;
          }
          noTone(5);
        }
}

void LoseGame(){
	//import Conrad's lose tone code
	int xLED = 0;
        int i=0;
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //for turning one LED on at a time then after displaying for a while, switch to next LED
			if(rowDISPLAY%(3*(columnDISPLAY%2+1))==0){ //0=red, 1=green, 2=blue
				xLED=1;
			}else{
				xLED=0;
			}
			LEDs[5][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[4][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[3][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[2][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[1][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[0][columnDISPLAY][rowDISPLAY]=xLED;
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
        int melody[] = {
          NOTE_A3, NOTE_G3,0, NOTE_G3, NOTE_G3,NOTE_F3, NOTE_C3, NOTE_A3, NOTE_E3,0,NOTE_E2,NOTE_C2,0};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {
          8, 8, 8, 8,4,4,4,8,8,8,8,8,8 };
        
        for (int thisNote = 0; thisNote < 13; thisNote++) {
       
          // to calculate the note duration, take one second 
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1000/noteDurations[thisNote];
          tone(5, melody[thisNote],noteDuration);
      
          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.30;
          i=0;
          while(i<(pauseBetweenNotes/7)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
            i++;
          }
          noTone(5);
        }
        
}	

void WinGame(){
        int xLED = 0;
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //for turning one LED on at a time then after displaying for a while, switch to next LED
			if(rowDISPLAY%(3*(columnDISPLAY%2+1))==2){ //0=red, 1=green, 2=blue
				xLED=1;
			}else{
				xLED=0;
			}
			LEDs[5][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[4][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[3][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[2][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[1][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[0][columnDISPLAY][rowDISPLAY]=xLED;
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
        int i = 0;
        int melody[] = {
          NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B5, NOTE_A5, NOTE_G5, NOTE_F5,NOTE_E5,NOTE_D5,NOTE_C5, NOTE_B6, NOTE_A6, NOTE_G6,NOTE_F6,NOTE_E6};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {
            8,8,8,4,4,4,8,8,8,8,8,8,8,8,8,8,8 };
        // iterate over the notes of the melody:
        for (int thisNote = 0; thisNote < 19; thisNote++) {
       
          // to calculate the note duration, take one second 
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1000/noteDurations[thisNote];
          tone(5, melody[thisNote],noteDuration);
      
          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.30;
          i=0;
          while(i<(pauseBetweenNotes/7)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
            i++;
          }
          // stop the tone playing:
          noTone(5);
        }    
}

void FriendedTone(){
	//import Conrad's friended tone code
        int i = 0;
        int melody[] = {
          NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B4, NOTE_A4, NOTE_G3, NOTE_F3,NOTE_E3,NOTE_D3,NOTE_C3, NOTE_B3, NOTE_A3, NOTE_G2,NOTE_F2,NOTE_E2};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {
          8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8, };
        for (int thisNote = 0; thisNote < 19; thisNote++) {
       
          // to calculate the note duration, take one second 
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1000/noteDurations[thisNote];
          tone(5, melody[thisNote],noteDuration);
      
          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.30;
          i=0;
          while(i<(pauseBetweenNotes/15)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
            i++;
          }
          noTone(5);
        }
	displayLEDs(LEDs);
}

void transmitGameState(){
    payload[30] = 64 + GameState;
    payload[31] = 65; //ID for F1 = 65, F2 = 66, F3 = 67
    xbee.send(tx);
}

void DisplayGame3(){
	updateaccel(x,y,z);
        int i = 0;
        int melody[] = {
          NOTE_F4,NOTE_D4, NOTE_C5};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {
          8, 8, 8};
	int xLED = 0;
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //for turning one LED on at a time then after displaying for a while, switch to next LED
			if(rowDISPLAY%3==2){ //0=red, 1=green, 2=blue
				xLED=1;
			}else{
				xLED=0;
			}
			LEDs[5][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[4][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[3][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[2][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[1][columnDISPLAY][rowDISPLAY]=xLED;
			LEDs[0][columnDISPLAY][rowDISPLAY]=xLED;
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
	displayLEDs(LEDs);
        for (int thisNote = 0; thisNote < 3; thisNote++) {
       
          // to calculate the note duration, take one second 
          // divided by the note type.
          //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
          int noteDuration = 1000/noteDurations[thisNote];
          tone(5, melody[thisNote],noteDuration);
      
          // to distinguish the notes, set a minimum time between them.
          // the note's duration + 30% seems to work well:
          int pauseBetweenNotes = noteDuration * 1.30;
          i=0;
          while(i<(pauseBetweenNotes/7)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
            i++;
          }
          noTone(5);
        }
}

int topFace(){
  //import Kents code
  return 0;
}

void Game3(){
	updateaccel(x,y,z);
        int i = 0;
        DisplayGame3();
        i = random(6);
        while(i == acceltop(x,y,z)){
          i = random(6);
        }
        columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //for turning one LED on at a time then after displaying for a while, switch to next LED
			LEDs[5][columnDISPLAY][rowDISPLAY]=LOW;
			LEDs[4][columnDISPLAY][rowDISPLAY]=LOW;
			LEDs[3][columnDISPLAY][rowDISPLAY]=LOW;
			LEDs[2][columnDISPLAY][rowDISPLAY]=LOW;
			LEDs[1][columnDISPLAY][rowDISPLAY]=LOW;
			LEDs[0][columnDISPLAY][rowDISPLAY]=LOW;
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
        TurnOnColumn(i,1,2);
        TurnOnColumn(i,2,2);
        TurnOnColumn(i,3,2);
        TurnOnColumn(i,4,2);
        TurnOnColumn(i,0,2);
        while(acceltop(x,y,z)!=i){
          displayLEDs(LEDs);
        }
        WinGame();
        GameState = 0;
} 

void Game2(){
	updateaccel(x,y,z);
        int i = 0;
        GameState = 20;
	DisplayGame2();
	transmitGameState();
        receivedGameState = 0;
	while(receivedGameState<20 || receivedGameState>26){ //wait until a partner is doing Game2
		displayLEDs(LEDs);
		xbee.readPacket();
		if (xbee.getResponse().isAvailable()) {  // got something
		   xbee.getResponse().getRx16Response(rx16);
		   receivedGameState = (int) (rx16.getData(30)&31); //where the color ID is, can be used for GameState too (why not?)
		}
                i++;
                if(i == 300){ // make sure other cube friends properly
                  transmitGameState();
                  i = 0;
                }
	}
	//GameState = 21 + random(6);
    transmitGameState();
	//GameState++;
	FriendedTone(); //show that a friend was found and the game can start now
	i = random(600)+100;
	while(i>0){ // wait for a random amount of time
		displayLEDs(LEDs);
		i--;
	}
		
	GameState = 25; //means that it's ready to ShowOff
	transmitGameState(); //tell the friend that it's ready to ShowOff
	i = 0;
	while(receivedGameState<23 || receivedGameState>27){ //wait until the friend is ready to ShowOff too
		displayLEDs(LEDs);
		xbee.readPacket();
		if (xbee.getResponse().isAvailable()) {  // got something
		   xbee.getResponse().getRx16Response(rx16);
		   receivedGameState = (int) (rx16.getData(30)&31); //where the color ID is, can be used for GameState too (why not?)
		}
		i++;
		if(i == (300)){ // make sure other cube friends properly
		  //GameState = 25;
		  transmitGameState();
		  i = 0;
		}
		Serial.println("stuck");
	}
	if(receivedGameState!=27){
		GameState = 27;//means that it's ready to ShowOff and it has a head start
		transmitGameState();
		i = 0;
		while(i<200){
			displayLEDs(LEDs);
			i++;
		}
		transmitGameState();
		i = 0;
		Serial.println("head start");
	}
	ShowOff(); //now that everyone is ready to go, ShowOff
	displayLEDs(LEDs);
	i = 0;
	while(receivedGameState!=29 && acceljerk(x,y,z)==0){ //no one has won yet
        ShowOff();
		displayLEDs(LEDs);
		//transmitGameState();
		xbee.readPacket();
		if (xbee.getResponse().isAvailable()) {  // got something
		   xbee.getResponse().getRx16Response(rx16);
		   receivedGameState = (int) (rx16.getData(30)&31); //where the color ID is, can be used for GameState too (why not?)
		}
		i++;
		if(i == (300)){ // make sure other cube friends properly
		  //GameState = 25;
		  transmitGameState();
		  i = 0;
		}
	}
        Serial.println("hey WIN OR LOSE?");
	if(receivedGameState == 29){
		LoseGame();
                Serial.println("LOSE");
	}else{
		GameState = 29;
        transmitGameState();
		//WinGame();
		i = 0;
		while(i<25){
			displayLEDs(LEDs);
			if(i%5==0){
				transmitGameState();
			}
			i++;
		}
		WinGame();
		displayLEDs(LEDs);
                Serial.println("WIN");
                //delay(10000);
	}
	GameState = 0; //go back to game selection screen
}

void Game1(){
	updateaccel(x,y,z);
	StartGame1();
	int solvedFaces[] = {6,6,6,6,6};
	int i = 0;
        Serial.println("game 1");
	while(GameState<18){ //GameState=18 for Lose, GameState=19 for Win
		Serial.println(GameState);
		int absFace = 0; //for storing the accelerometer result
		int absColumn = 2;
		int absRow = 2;
		int memFace = 0; //for turning off the LED where you were a bit ago
		int memColumn = 2;
		int memRow = 2;
		absFace = acceltop(x, y, z); //Kent's accelerometer code
		if(absFace!=0 && absFace!=1 && absFace!=3 ){ //if you've ate an apple
			if(solvedFaces[0]!=absFace && solvedFaces[1]!=absFace && solvedFaces[2]!=absFace && solvedFaces[3]!=absFace && solvedFaces[4]!=absFace){
				GameState++; //ate an apple, advance in game
				solvedFaces[i] = absFace; // record that that face was solved
				i++;
				if(i==3){ //you've ate all the apples! you win
					GameState = 19;//win
				}
			}
		}
		if(absFace==1 || absFace==3){ //you've gone out of bounds, lose
			GameState = 18;//you lose
                        Serial.println("hey1");
		}/*else if((absFace==4 || absFace==2) && (absRow==1 || absRow==5)){//you've hit the rails, lose
			GameState = 18;//you lose
                        Serial.println("hey2");
		}else if((absFace==5 || absFace==0) && (absColumn==1 || absColumn==5)){//you've hit the rails, lose
			GameState = 18;//you lose
                        Serial.println("hey3");
		}*/
		TurnOffSingleLED(memFace,3,3); //turn off the LED where you were last
		TurnOnSingleLED(absFace,3,3, 2); //turn on blue LED where you are
		memFace = absFace;
		displayLEDs(LEDs);
	}
        Serial.println(GameState);
	if(GameState==18){
		LoseGame();
	}else if(GameState==19){
		WinGame();
	}
	GameState = 0;
}

int idle(){
	int i = 0;
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //turn all LEDs off
			faceDISPLAY = 0;
			while(faceDISPLAY<6){
				LEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
				faceDISPLAY++;
			}
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
	if(acceljerk(x,y,z)==1){
		TurnOnSingleLED(0,3,1,2);//top face show #1
		TurnOnSingleLED(0,3,2,2);
		TurnOnSingleLED(0,3,3,2);
		TurnOnSingleLED(0,3,4,2);
		TurnOnSingleLED(0,3,5,2);
		TurnOnSingleLED(4,2,1,2);//front face show #2
		TurnOnSingleLED(4,3,1,2);
		TurnOnSingleLED(4,4,1,2);
		TurnOnSingleLED(4,4,2,2);
		TurnOnSingleLED(4,4,3,2);
		TurnOnSingleLED(4,3,3,2);
		TurnOnSingleLED(4,2,3,2);
		TurnOnSingleLED(4,2,4,2);
		TurnOnSingleLED(4,2,5,2);
		TurnOnSingleLED(4,3,5,2);
		TurnOnSingleLED(4,4,5,2);
		TurnOnSingleLED(3,2,1,2);//front face show #3
		TurnOnSingleLED(3,3,1,2);
		TurnOnSingleLED(3,4,1,2);
		TurnOnSingleLED(3,4,2,2);
		TurnOnSingleLED(3,4,3,2);
		TurnOnSingleLED(3,3,3,2);
		TurnOnSingleLED(3,2,3,2);
		TurnOnSingleLED(3,4,4,2);
		TurnOnSingleLED(3,4,5,2);
		TurnOnSingleLED(3,3,5,2);
		TurnOnSingleLED(3,2,5,2);	
		displayLEDs(LEDs);
		while(i<300){
			displayLEDs(LEDs);
			i++;
		}
			
		return 0;
		
	}else{
		return -1;
	}
	displayLEDs(LEDs);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int xLED = 0;
int yLED = 0;
int loc0 = 0;

int TestCount = 0;
//GameState = 20;
void loop() {
	while(1){
		ShowOffAccel();
		displayLEDs(LEDs);
	}
	int i = 0;
	
	if(GameState == -1){   
		GameState = idle();
	}
	if(GameState == 0){   
		SelectGame();
	}
	if(GameState == 10){
		Game1();
	}else if(GameState == 20){
		Game2();
	}else if(GameState == 30){
		Game3();
	}
	//Serial.println("hey");
	displayLEDs(LEDs);
	
}
