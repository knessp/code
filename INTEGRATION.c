#include <XBee.h>
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
boolean fakeLEDs[6][5][16]; 
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
int SleepIndex = 0;
int SleepLog[100];
const int ZeroG = 13;
const int read_accel_x = 14;
const int read_accel_y = 15;
const int read_accel_z = 16;
const int read_averaging = 5000;/*determines how many readings the ADC averages before sending to cpu*/ //doesn't seem like it works to me (Phil)
const int read_smoothing = 10;/*determines how 'smooth' the readings are*/
const int wiggle = 100;/*determines wiggle room in determining what side is on top */
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
const int Hysteresis = 30; //the step size of the accelerometer you want... (For battling jitteryness! Jitteryness is the enemy!)
// Range is about 480 on each axis, 10 LEDs between one side and another. So Hysteresis = 48ish or 40 to 25ish seems like a good idea. 10 too low.
// I think 25 is good, it only gets jittery if the user has jittery hands. So maybe do 30.

/*GAME STUFF: */
int GameState = -1;

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
  Serial3.begin(115200);
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
  displayLEDs(LEDs);
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

void updateaccelOTHER(){ //adds the reading from the accelerometer to the previous readings. Used in displayLEDs() function to always be averaging nicely.
	x_average_value = x_average_value + analogRead(read_accel_x); //just adding it to a running score
	y_average_value = y_average_value + analogRead(read_accel_y);
	z_average_value = z_average_value + analogRead(read_accel_z);
	accel_points_taken++;	//and increments the number of times it's added to the total so that it can be averaged when you want to know the average.
}

void accelAveWithHysteresis(){ //rounds-ish (it's sticky) the x,y,z values to the nearest (Hysteresis value) based on previous value
	int tempX = x_hysteresis;
	int tempY = y_hysteresis;
	int tempZ = z_hysteresis;
	if(x_hysteresis==0){	//for first time use: just round normally
		if((x_average_value%Hysteresis)<(Hysteresis/2)){	//round down
			x_hysteresis = x_average_value - (x_average_value%Hysteresis);
			y_hysteresis = y_average_value - (y_average_value%Hysteresis);
			z_hysteresis = z_average_value - (z_average_value%Hysteresis);
		}else{		//round up
			x_hysteresis = x_average_value + (Hysteresis-(x_average_value%Hysteresis));
			y_hysteresis = y_average_value + (Hysteresis-(y_average_value%Hysteresis));
			z_hysteresis = z_average_value + (Hysteresis-(z_average_value%Hysteresis));
		}
	}else{		//keeps it to previous value unless it's a significant amount higher or lower
		if(x_average_value > x_hysteresis){  // X HYSTERESIS VALUE
			if((x_average_value - x_hysteresis)>((Hysteresis/2)+(Hysteresis/4))){ //significantly higher
				x_hysteresis = x_hysteresis + Hysteresis;
			}
		}else{ //smaller
			if((x_hysteresis - x_average_value)>((Hysteresis/2)+(Hysteresis/4))){ //significantly lower
				x_hysteresis = x_hysteresis - Hysteresis;
			}
		}
		if(y_average_value > y_hysteresis){  // Y HYSTERESIS VALUE
			if((y_average_value - y_hysteresis)>((Hysteresis/2)+(Hysteresis/4))){ //significantly higher
				y_hysteresis = y_hysteresis + Hysteresis;
			}
		}else{ //smaller
			if((y_hysteresis - y_average_value)>((Hysteresis/2)+(Hysteresis/4))){ //significantly lower
				y_hysteresis = y_hysteresis - Hysteresis;
			}
		}
		if(z_average_value > z_hysteresis){  // Z HYSTERESIS VALUE
			if((z_average_value - z_hysteresis)>((Hysteresis/2)+(Hysteresis/4))){ //significantly higher
				z_hysteresis = z_hysteresis + Hysteresis;
			}
		}else{ //smaller
			if((z_hysteresis - z_average_value)>((Hysteresis/2)+(Hysteresis/4))){ //significantly lower
				z_hysteresis = z_hysteresis - Hysteresis;
			}
		} //if not significant change, don't change the hysteresis value
			//NOTE: this assumes the values don't change by more than 18 at a time. Fix?
	}	
	// if((tempX!=x_hysteresis)||(tempY!=y_hysteresis)||(tempZ!=z_hysteresis)){ //just for testing
		// Serial.print("X hysteresis: ");
		// Serial.print(x_hysteresis);
		// Serial.print(" Y hysteresis: ");
		// Serial.print(y_hysteresis);
		// Serial.print(" Z hysteresis: ");
		// Serial.println(z_hysteresis);
	// }
}

void accelaveOTHER(){ //from Phil. takes the sum of the readings and divides it by how many readings there were to get the average
	x_average_value = x_average_value/accel_points_taken; //takes the average of the running tally when called and stores it in global variable
	y_average_value = y_average_value/accel_points_taken;
	z_average_value = z_average_value/accel_points_taken;
	accel_points_taken = 0;
	accelAveWithHysteresis(); //also calls hysteresis function in case you want to use hysteresis values (You Should (no jitters!))
	//after get numbers, do clearaccelOTHER()!!!!!!!!!!!!
}

void clearaccelOTHER(){ //from Phil. use this after accelaveOTHER to reset the averaging process
	x_average_value = 0;
	y_average_value = 0;
	z_average_value = 0;
}

int GoToSleep(){
	int sleepTime = 7;  //30 corresponds to about 30 seconds. Number determines how fast it will go to sleep with inactivity
	int sleepSensitivity = 27; //Determines how active you have to be to avoid going to asleep
	int maxSleep;
	int minSleep;
	int i = 0;
	if(SleepIndex%sleepTime == 0){ 
		SleepLog[SleepIndex/sleepTime] = (int) analogRead(read_accel_x) + (int) analogRead(read_accel_y);
	}
	if(SleepIndex == 99*sleepTime){
		//Serial.println("Sleep?");
		SleepIndex = 0;
		maxSleep = SleepLog[0];
		minSleep = SleepLog[0];
		while(i<100){
			if(maxSleep < SleepLog[i]){
				maxSleep = SleepLog[i];
			}
			if(minSleep > SleepLog[i]){
				minSleep = SleepLog[i];
			}
			i++;
		}
		//Serial.println(minSleep);
		// Serial.println(maxSleep-minSleep);
		if((maxSleep-minSleep) < sleepSensitivity){
			Serial.println("GGOOO TTOOOO SSSLLLEEEEEEEEEEEEEEEEEEEEEEPPPPP!!!!!!!!");
			return 1;
		}
	}else{
		SleepIndex++;
	}
	return 0;
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
  accelaveOTHER();
  xval = x_average_value;
  yval = y_average_value;
  zval = z_average_value;
  clearaccelOTHER();
  // Serial.print("X: ");
  // Serial.print(xval);
  // Serial.print(" Y: ");
  // Serial.print(yval);
  // Serial.print(" Z: ");
  // Serial.println(zval);
  
  if(xval > (250 - wiggle) && xval < (250 + wiggle) && yval > (533 - wiggle) && yval < (533 + wiggle) && zval > (435 - wiggle) && zval < (435 + wiggle)){
    return 0;//f
  }
  else if(xval > (481 - wiggle) && xval < (481 + wiggle) && yval > (530 - wiggle) && yval < (530 + wiggle) && zval > (191 - wiggle) && zval < (191 + wiggle)){
    return 4;//f
  }
  else if(xval > (478 - wiggle) && xval < (478 + wiggle) && yval > (291 - wiggle) && yval < (291 + wiggle) && zval > (434 - wiggle) && zval < (434 + wiggle)){
    return 1;//f
  }
  else if(xval > (487 - wiggle) && xval < (487 + wiggle) && yval > (775 - wiggle) && yval < (775 + wiggle) && zval > (428 - wiggle) && zval < (428 + wiggle)){
    return 3;//f
  }
  else if(xval > (496 - wiggle) && xval < (496 + wiggle) && yval > (539 - wiggle) && yval < (539 + wiggle) && zval > (667 - wiggle) && zval < (667 + wiggle)){
    return 2;//f
  }
  else if(xval > (725 - wiggle) && xval < (725 + wiggle) && yval > (529 - wiggle) && yval < (529 + wiggle) && zval > (428 - wiggle) && zval < (428 + wiggle)){
    return 5;//f
  }
  else{
    return 6; //means in-between faces
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

void displayLEDs(boolean LED[6][5][16]){ //needs to be called often to display the LEDs without flickering. Use this for delay instead of delay().

  digitalWrite(clk,0);
  digitalWrite(sdi,LOW);
    
  columnDISPLAY=0;
  while(columnDISPLAY<5){    //for displaying each column
     // digitalWriteFast(clk,1);         //Prime the clock    
     digitalWriteFast(clk,0);
     digitalWriteFast(le,HIGH);         //Begin
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
     digitalWriteFast(oe,LOW);
     
     digitalWrite(Mosfet[columnDISPLAY],0);       //turn the correct column on
     delayMicroseconds(t);        //keep the column on for a little while
     //xbee.send(tx);	
	 digitalWrite(Mosfet[columnDISPLAY], 1);  //then turn the mosfet off so we can move to the next column
     updateaccelOTHER();     								/*KENT YOU CAN CHANGE THIS TO PUSH AND POP*/
	 //delayMicroseconds(1);
     //updateaccelOTHER();     								/*KENT YOU CAN CHANGE THIS TO PUSH AND POP*/
	 columnDISPLAY++;                         //move to the next column
  }
	if(GoToSleep()){
		GameState = -1;
		//loop();
	}
}

void ShowTransmit(boolean LED[6][5][16]){ //Called when transmitting when you want to be flashy about it
  digitalWrite(clk,0);
  digitalWrite(sdi,LOW);
  columnDISPLAY=0;
  while(columnDISPLAY<5){    //for displaying each column
     digitalWriteFast(clk,0);
     digitalWriteFast(le,HIGH);         //Begin
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
     digitalWriteFast(oe,LOW);
     digitalWriteFast(Mosfet[columnDISPLAY],0);       //turn the correct column on
	 delayMicroseconds(10000);
	 xbee.send(tx);					//keep the column on for a little while
     digitalWriteFast(Mosfet[columnDISPLAY], 1);  //then turn the mosfet off so we can move to the next column
	 columnDISPLAY++;                         //move to the next column
  }
  //Serial.println("transmitted");
}

void TurnOnSingleLED(int face, int column, int row, int color){//red=0,green=1,blue=2, rg=3, rb=4, gb = 5, all=6. Row: 1-5. Column: 1-5. Face: 0-5.
	int row1 = (row)*3;
	int row2 = (row)*3-1;
	int row3 = (row)*3-2;
	row = (row)*3-color;
	if(color == 0){
		LEDs[face][column-1][row] = HIGH;
		LEDs[face][column-1][row2] = LOW;
		LEDs[face][column-1][row3] = LOW;
	}else if(color == 1){
		LEDs[face][column-1][row] = HIGH;
		LEDs[face][column-1][row1] = LOW;
		LEDs[face][column-1][row3] = LOW;
	}else if(color == 2){
		LEDs[face][column-1][row] = HIGH;
		LEDs[face][column-1][row2] = LOW;
		LEDs[face][column-1][row1] = LOW;
	}else if(color == 3){
		LEDs[face][column-1][row1] = HIGH;
		LEDs[face][column-1][row2] = HIGH;
		LEDs[face][column-1][row3] = LOW;
	}else if(color == 4){
		LEDs[face][column-1][row1] = HIGH;
		LEDs[face][column-1][row2] = LOW;
		LEDs[face][column-1][row3] = HIGH;
	}else if(color == 5){
		LEDs[face][column-1][row1] = LOW;
		LEDs[face][column-1][row2] = HIGH;
		LEDs[face][column-1][row3] = HIGH;
	}else if(color == 6){
		LEDs[face][column-1][row1] = HIGH;
		LEDs[face][column-1][row2] = HIGH;
		LEDs[face][column-1][row3] = HIGH;
	}
}

void TurnOffSingleLED(int face, int column, int row){ //Row: 1-5. Column: 1-5. Face: 0-5.
	row = (row)*3;
	LEDs[face][column-1][row] = LOW;
	LEDs[face][column-1][row-1] = LOW;
	LEDs[face][column-1][row-2] = LOW;
}

void TurnOnRow(int face, int row, int color){//red=0,green=1,blue=2. Row: 1-5. Face: 0-5.
  int row1 = (row)*3;
  int row2 = (row)*3-1;
  int row3 = (row)*3-2;
  row = (row)*3-color;
  int column = 0;
  while(column<5){
    LEDs[face][column][row1] = LOW;
    LEDs[face][column][row2] = LOW;
    LEDs[face][column][row3] = LOW;
    LEDs[face][column][row] = HIGH;
    column++;
  }
}

void TurnOffRow(int face, int row){ // Row: 1-5. Face: 0-5.
  int row1 = (row)*3;
  int row2 = (row)*3-1;
  int row3 = (row)*3-2;
  int column = 0;
  while(column<5){
    LEDs[face][column][row1] = LOW;
    LEDs[face][column][row2] = LOW;
    LEDs[face][column][row3] = LOW;
    column++;
  }
}

void TurnOnColumn(int face, int column, int color){//red=0,green=1,blue=2. Column: 1-5. Face: 0-5.
  int row = 0;
  while(row<16){
    LEDs[face][column-1][row] = LOW;
    row++;
  }
  row = 3-color;
  LEDs[face][column-1][row] = HIGH;
  row = row+3;
  LEDs[face][column-1][row] = HIGH;
  row = row+3;
  LEDs[face][column-1][row] = HIGH;
  row = row+3;
  LEDs[face][column-1][row] = HIGH;
  row = row+3;
  LEDs[face][column-1][row] = HIGH;
}

void TurnOffColumn(int face, int column){// Column: 1-5. Face: 0-5.
	int row = 0;
	while(row<16){
		LEDs[face][column-1][row] = LOW;
		row++;
	}
}

void TurnOnFace(int face, int color){//red=0,green=1,blue=2, rg=3, rb=4, gb = 5, all=6. Face: 0-5.
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

void ShowOffAccel(){ //kinda lame but shows accel working well
	int row = 1;
	int column = 1;
	int color = 0;
	accelaveOTHER();
	int AccelOffSet = z_hysteresis/50; //can do any axis
	clearaccelOTHER();
	//Serial.println(AccelOffSet);
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

void WhiteColor(){ //just for experimenting
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

void MoveLED(int location[], int movePitch, int moveYaw, int moveRoll){ //WORKS NOW (i'm pretty sure)!! 0=Face: 0-5. 1=Column: 1-5. 2=Row: 1-5. 
	int face = location[0];
	int column = location[1];
	int row = location[2];
	if(movePitch!=0&&moveYaw!=0){
		MoveLED(location,movePitch,0,moveRoll);
		movePitch = 0;
		face = location[0];
		column = location[1];
		row = location[2];
	}
	if(movePitch!=0&&moveRoll!=0){
		MoveLED(location, movePitch,moveYaw,0);
		movePitch = 0;
		face = location[0];
		column = location[1];
		row = location[2];
	}
	if(moveYaw!=0&&moveRoll!=0){
		MoveLED(location, movePitch,moveYaw,0);
		moveYaw = 0;
		face = location[0];
		column = location[1];
		row = location[2];
	}
	int color=7;
	int tempRow = row*3;
	if(LEDs[face][column-1][tempRow]==1){
		color = 0;
	}else if(LEDs[face][column-1][tempRow-1]==1){
		color = 1;
	}else if(LEDs[face][column-1][tempRow-2]==1){
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
	if(color==7){
		Serial.println("hey! color=7, moveLED");
		color = 0;
	}
	if(face == 5){ //+column is -Roll, +row is -Pitch
		TurnOffSingleLED(face, column, row);
		if(column-moveRoll>5){
			face = 1;
			moveRoll = (column-moveRoll)-6;
			column = -(row-6);
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch>5){
			face = 4;
			movePitch = (row-movePitch)-6;
			row = -(column-6);
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		if(column-moveRoll<1){
			face = 3;
			moveRoll = column-moveRoll;
			column = 6-row;
			row = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch<1){
			face = 2;
			movePitch = row-movePitch;
			row = column;
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column-moveRoll, row-movePitch, color);
                location[0] = face; //for telling you where the LED ended up after you moved it
                location[1] = column-moveRoll;
                location[2] = row-movePitch;                
	}else if(face == 4){ //+column is -Pitch, +row is -Yaw
		TurnOffSingleLED(face, column, row);
		if(column-movePitch>5){
			face = 0;
			movePitch = (column-movePitch)-6;
			column = (6-row);
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveYaw>5){
			face = 3;
			moveYaw = (row-moveYaw)-6;
			row = -(column-6);
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		if(column-movePitch<1){
			face = 5;
			movePitch = column-movePitch;
			column = 6-row;
			row = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveYaw<1){	 
			face = 1;
			moveYaw = row-moveYaw;
			row = column;
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column-movePitch, row-moveYaw, color);	
                location[0] = face;
                location[1] = column-movePitch;
                location[2] = row-moveYaw;       
	}else if(face == 3){ //+column is -Yaw, +row is -Roll
		TurnOffSingleLED(face, column, row);
		if(column-moveYaw>5){
			face = 2;
			moveYaw = (column-moveYaw)-6;
			column = (6-row);
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll>5){
			face = 5;
			moveRoll = (row-moveRoll)-6;
			row = 6-column;
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		if(column-moveYaw<1){
			face = 4;
			moveYaw = column-moveYaw;
			column = 6-row;
			row = 5;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll<1){
			face = 0;
			moveRoll = row-moveRoll;
			row = column;
			column = 1;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column-moveYaw, row-moveRoll, color);
                location[0] = face;
                location[1] = column-moveYaw;
                location[2] = row-moveRoll;       
	}else if(face == 2){ //+column is +Pitch, +row is -Yaw	 
		TurnOffSingleLED(face, column, row);
		if(column+movePitch>5){
			face = 0;
			movePitch = (column+movePitch)-6;
			column = (row);
			row = 5;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveYaw>5){
			face = 1;
			moveYaw = (row-moveYaw)-6;
			row = column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		if(column+movePitch<1){
			face = 5;
			movePitch = column+movePitch;
			column = row;
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveYaw<1){
			face = 3;
			moveYaw = row-moveYaw;
			row = -(column-6);
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+movePitch, row-moveYaw, color);
                location[0] = face;
                location[1] = column+movePitch;
                location[2] = row-moveYaw;       
	}else if(face == 1){ //+column is +Yaw, +row is -Roll
		TurnOffSingleLED(face, column, row);
		if(column+moveYaw>5){
			face = 2;
			moveYaw = (column+moveYaw)-6;
			column = row;
			row = 5; 	 
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll>5){
			face = 0;
			moveRoll = (row-moveRoll)-6;
			row = column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		if(column+moveYaw<1){
			face = 4;
			moveYaw = column+moveYaw;
			column = row;
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-moveRoll<1){
			face = 5;
			moveRoll = row-moveRoll;
			row = 6-column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+moveYaw, row-moveRoll, color);
                location[0] = face;
                location[1] = column+moveYaw;
                location[2] = row-moveRoll;       
	}else if(face == 0){ //(+column is +Pitch, +row is +Roll!) +column is +Roll, +row is -Pitch
		TurnOffSingleLED(face, column, row);
		if(column+moveRoll>5){
			face = 1;
			moveRoll = (column+moveRoll)-6;
			column = row;
			row = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch>5){
			face = 2;
			movePitch = (row-movePitch)-6;
			row = column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		if(column+moveRoll<1){
			face = 3;
			moveRoll = column+moveRoll;
			column = row;
			row = 1;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}else if(row-movePitch<1){
			face = 4;
			movePitch = row-movePitch;
			row = 6-column;
			column = 5;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			MoveLED(location, movePitch, moveYaw, moveRoll);
		}
		TurnOnSingleLED(face, column+moveRoll, row-movePitch, color);
		location[0] = face;
		location[1] = column+moveRoll;
		location[2] = row-movePitch;       
	}
} 

void TrailLED(int location[], int movePitch, int moveYaw, int moveRoll, int color){ //same as MoveLED but keeps original LED
	int face = location[0];
	int column = location[1];
	int row = location[2];
	int save;
	if(movePitch!=0&&moveYaw!=0){
		save = movePitch;
		TrailLED(location,0,moveYaw,moveRoll,color);
		moveYaw = 0;
		movePitch = save;
		face = location[0];
		column = location[1];
		row = location[2];
	}
	if(movePitch!=0&&moveRoll!=0){
		save = moveRoll;
		TrailLED(location, movePitch,moveYaw,0,color);
		movePitch = 0;
		moveRoll = save;
		face = location[0];
		column = location[1];
		row = location[2];
	}
	if(moveYaw!=0&&moveRoll!=0){
		save = moveYaw;
		TrailLED(location, movePitch,0,moveRoll,color);
		moveRoll = 0;
		moveYaw = save;
		face = location[0];
		column = location[1];
		row = location[2];
	}
	int tempRow = row*3;
	if(face == 5){ //+column is -Roll, +row is -Pitch
		if(column-moveRoll>5){
			face = 1;
			moveRoll = (column-moveRoll)-6;
			column = -(row-6);
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-movePitch>5){
			face = 4;
			movePitch = (row-movePitch)-6;
			row = -(column-6);
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		if(column-moveRoll<1){
			face = 3;
			moveRoll = column-moveRoll;
			column = 6-row;
			row = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-movePitch<1){
			face = 2;
			movePitch = row-movePitch;
			row = column;
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		TurnOnSingleLED(face, column-moveRoll, row-movePitch, color);
		location[0] = face; //for telling you where the LED ended up after you moved it
		location[1] = column-moveRoll;
		location[2] = row-movePitch;                
	}else if(face == 4){ //+column is -Pitch, +row is -Yaw
		if(column-movePitch>5){
			face = 0;
			movePitch = (column-movePitch)-6;
			column = (6-row);
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveYaw>5){
			face = 3;
			moveYaw = (row-moveYaw)-6;
			row = -(column-6);
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		if(column-movePitch<1){
			face = 5;
			movePitch = column-movePitch;
			column = 6-row;
			row = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveYaw<1){	 
			face = 1;
			moveYaw = row-moveYaw;
			row = column;
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		TurnOnSingleLED(face, column-movePitch, row-moveYaw, color);	
		location[0] = face;
		location[1] = column-movePitch;
		location[2] = row-moveYaw;       
	}else if(face == 3){ //+column is -Yaw, +row is -Roll
		if(column-moveYaw>5){
			face = 2;
			moveYaw = (column-moveYaw)-6;
			column = 6-row;
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveRoll>5){
			face = 5;
			moveRoll = (row-moveRoll)-6;
			row = 6-column;
			column = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		if(column-moveYaw<1){
			face = 4;
			moveYaw = column-moveYaw;
			column = 6-row;
			row = 5;	 
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveRoll<1){
			face = 0;
			moveRoll = row-moveRoll;
			row = column;
			column = 1;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		TurnOnSingleLED(face, column-moveYaw, row-moveRoll, color);
		location[0] = face;
		location[1] = column-moveYaw;
		location[2] = row-moveRoll;       
	}else if(face == 2){ //+column is +Pitch, +row is -Yaw	 
		if(column+movePitch>5){
			face = 0;
			movePitch = (column+movePitch)-6;
			column = (row);
			row = 5;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveYaw>5){
			face = 1;
			moveYaw = (row-moveYaw)-6;
			row = column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		if(column+movePitch<1){
			face = 5;
			movePitch = column+movePitch;
			column = row;
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveYaw<1){
			face = 3;
			moveYaw = row-moveYaw;
			row = -(column-6);
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		TurnOnSingleLED(face, column+movePitch, row-moveYaw, color);
		location[0] = face;
		location[1] = column+movePitch;
		location[2] = row-moveYaw;       
	}else if(face == 1){ //+column is +Yaw, +row is -Roll
		if(column+moveYaw>5){
			face = 2;
			moveYaw = (column+moveYaw)-6;
			column = row;
			row = 5; 	 
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveRoll>5){
			face = 0;
			moveRoll = (row-moveRoll)-6;
			row = column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		if(column+moveYaw<1){
			face = 4;
			moveYaw = column+moveYaw;
			column = row;
			row = 1;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-moveRoll<1){
			face = 5;
			moveRoll = row-moveRoll;
			row = 6-column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		TurnOnSingleLED(face, column+moveYaw, row-moveRoll, color);
		location[0] = face;
		location[1] = column+moveYaw;
		location[2] = row-moveRoll;       
	}else if(face == 0){ //+column is +Pitch, +row is +Roll 	
		if(column+moveRoll>5){
			face = 1;
			moveRoll = (column+moveRoll)-6;
			column = row;
			row = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-movePitch>5){
			face = 2;
			movePitch = (row-movePitch)-6;
			row = column;
			column = 5;
			location[0] = face; 
            location[1] = column;
            location[2] = row; 
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		if(column+moveRoll<1){
			face = 3;
			moveRoll = column+moveRoll;
			column = row;
			row = 1;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}else if(row-movePitch<1){
			face = 4;
			movePitch = row-movePitch;
			row = 6-column;
			column = 5;
			location[0] = face;
            location[1] = column;
            location[2] = row;
			TurnOnSingleLED(location[0], location[1], location[2], color);
			TrailLED(location, movePitch, moveYaw, moveRoll,color);
		}
		TurnOnSingleLED(face, column+moveRoll, row-movePitch, color);
		location[0] = face;
		location[1] = column+moveRoll;
		location[2] = row-movePitch;        
	}
}

void CreateBlock(int location[], int color){
	int face = location[0];
	int column = location[1];
	int row = location[2];
	TurnOnSingleLED(face, column, row, color);
	
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,-1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,0,1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,0,-1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,-1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,0,0,1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,0,0,-1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,0,1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,0,-1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,-1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,-1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,0,1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,0,-1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,-1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,0,0,1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,0,0,-1,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,-1,0,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,0,1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,0,-1,0,color);
	location[0] = face;
	location[1] = column;
	location[2] = row;
}

void TurnBlockOff(int location[]){
	int face = location[0];
	int column = location[1];
	int row = location[2];
	int color = 7;
	
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,-1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,0,1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,1,0,0,color);
	TrailLED(location,0,0,-1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,-1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,0,0,1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TrailLED(location,0,0,-1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,0,1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,0,-1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TrailLED(location,-1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,-1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,0,1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TrailLED(location,0,0,-1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,-1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,0,0,1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TrailLED(location,0,0,-1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,-1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,0,1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TrailLED(location,0,-1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;

	TrailLED(location,1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,-1,0,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,-1,0,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;
	TrailLED(location,0,0,-1,color);
	TurnOffSingleLED(location[0], location[1], location[2]);
	location[0] = face;
	location[1] = column;
	location[2] = row;

	TurnOffSingleLED(face, column, row);
}

void MoveBlock(int location[], int movePitch, int moveYaw, int moveRoll, int color){//location[] contains the face, column, row information of the center of the LEDs that need moved
	int face = location[0];
	int column = location[1];
	int row = location[2];
	TurnBlockOff(location);
	TurnOnSingleLED(face,column,row,color);
	MoveLED(location, movePitch, moveYaw, moveRoll); //move center LED
	CreateBlock(location,color); //now create a new block around the new center
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

void TurnOnRowFake(int face, int row, int color){//red=0,green=1,blue=2. Row: 1-5. Face: 0-5.
  int row1 = (row)*3;
  int row2 = (row)*3-1;
  int row3 = (row)*3-2;
  row = (row)*3-color;
  int column = 0;
  while(column<5){
    //fakeLEDs[face][column][row1] = LOW;
    //fakeLEDs[face][column][row2] = LOW;
    //fakeLEDs[face][column][row3] = LOW;
    fakeLEDs[face][column][row] = HIGH;
    column++;
  }
}

void TurnOnColumnFake(int face, int column, int color){//red=0,green=1,blue=2. Column: 1-5. Face: 0-5.
  int row = 0;
  //while(row<16){
    //fakeLEDs[face][column-1][row] = LOW;
    //row++;
  // }
  row = 3-color;
  fakeLEDs[face][column-1][row] = HIGH;
  row = row+3;
  fakeLEDs[face][column-1][row] = HIGH;
  row = row+3;
  fakeLEDs[face][column-1][row] = HIGH;
  row = row+3;
  fakeLEDs[face][column-1][row] = HIGH;
  row = row+3;
  fakeLEDs[face][column-1][row] = HIGH;
}

void TurnOffCubeFake(){
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
		rowDISPLAY = 0;
		while(rowDISPLAY<16){      //turn all fakeLEDs off
			faceDISPLAY = 0;
			while(faceDISPLAY<6){
				fakeLEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]=LOW;
				faceDISPLAY++;
			}
			rowDISPLAY++;
		}
		columnDISPLAY++;
	}
}

void AddOnLED(int face, int column, int row, int color){//does not over-write colors that were already on for a given LED!! red=0,green=1,blue=2, rg=3, rb=4, gb = 5, all=6. Row: 1-5. Column: 1-5. Face: 0-5.
	int row1 = (row)*3;
	int row2 = (row)*3-1;
	int row3 = (row)*3-2;
	row = (row)*3-color;
	if(color == 0){
		fakeLEDs[face][column-1][row] = HIGH;
	}else if(color == 1){
		fakeLEDs[face][column-1][row] = HIGH;
	}else if(color == 2){
		fakeLEDs[face][column-1][row] = HIGH;
	}else if(color == 3){
		fakeLEDs[face][column-1][row1] = HIGH;
		fakeLEDs[face][column-1][row2] = HIGH;
	}else if(color == 4){
		fakeLEDs[face][column-1][row1] = HIGH;
		fakeLEDs[face][column-1][row3] = HIGH;
	}else if(color == 5){
		fakeLEDs[face][column-1][row2] = HIGH;
		fakeLEDs[face][column-1][row3] = HIGH;
	}else if(color == 6){
		fakeLEDs[face][column-1][row1] = HIGH;
		fakeLEDs[face][column-1][row2] = HIGH;
		fakeLEDs[face][column-1][row3] = HIGH;
	}
}

void topPosition(int location[]){ //modifies location to hold the "top LED". location[]: 0=Face:0-5, 1=Column:1-5, 2=Row:1-5. 
	TurnOffCubeFake();
	//TurnOffCubeLED();
	//TurnOffSingleLED(location[0],location[1],location[2]);
	float xval;
	float yval;
	float zval;
	int LEDcolor = 2;
	int LEDcolor2 = 1;
	int LEDcolor3 = 0;
	accelaveOTHER();
	xval = x_hysteresis;
	yval = y_hysteresis;
	zval = z_hysteresis;
	// xval = x_average_value;
	// yval = y_average_value;
	// zval = z_average_value;
	clearaccelOTHER();
  float xpr = 250;
  float ypr = 285;
  float zpr = 180;
  float Xdegree=(xval-xpr); //.374
  float Ydegree=(yval-ypr);
  float Zdegree=(zval-zpr);
  if(Xdegree<79){
	Xdegree = Xdegree*.635; //.577, .635
  }else if(Xdegree>78 && Xdegree<411){
	Xdegree = ((Xdegree-78)*.3)+45; //.271, .3
  }else if(Xdegree>410){
	Xdegree = ((Xdegree-410)*.739)+135; //.672, .739
  }
  if(Xdegree<0){
	Xdegree = 0;
  }else if(Xdegree>180){
	Xdegree = 180;
  }
  if(Ydegree<63){
	Ydegree = Ydegree*.8; //.726, .8
  }else if(Ydegree>62 && Ydegree<409){
	Ydegree = ((Ydegree-62)*.286)+45; //.260, .286
  }else if(Ydegree>408){
	Ydegree = ((Ydegree-408)*.619)+135; //.563, .619
  }
  if(Ydegree<0){
	Ydegree = 0;
  }else if(Ydegree>180){
	Ydegree = 180;
  }
  if(Zdegree<80){
	Zdegree = Zdegree*.627; //.570, .627
  }else if(Zdegree>79 && Zdegree<417){
	Zdegree = ((Zdegree-79)*.294)+45; //.267, .294
  }else if(Zdegree>416){
	Zdegree = ((Zdegree-416)*.839)+135; //.763, .839
  }
  if(Zdegree<0){
	Zdegree = 0;
  }else if(Zdegree>180){
	Zdegree = 180;
  }
  // Serial.print(" Xval: ");
  // Serial.print(xval);
  // Serial.print(" Yval: ");
  // Serial.print(yval);
  // Serial.print(" Zval: ");
  // Serial.print(zval);
  // Serial.print(" X: ");
  // Serial.print(Xdegree);
  // Serial.print(" Y: ");
  // Serial.print(Ydegree);
  // Serial.print(" Z: ");
  // Serial.print(Zdegree);
  // Serial.println(" ");
  
  if (Xdegree < 18-9){ //16.36
    AddOnLED(0,3,3,LEDcolor);
  }
  else if((Xdegree >= 18-9) && (Xdegree < (18*2-9))){
    AddOnLED(0,4,4,LEDcolor);
    AddOnLED(0,4,3,LEDcolor);
    AddOnLED(0,4,2,LEDcolor);
    AddOnLED(0,3,4,LEDcolor);
    AddOnLED(0,3,2,LEDcolor);
    AddOnLED(0,2,4,LEDcolor);
    AddOnLED(0,2,3,LEDcolor);
    AddOnLED(0,2,2,LEDcolor);
  }
  else if((Xdegree >= 18*2-9) && Xdegree < (18*3-9)){
    TurnOnRowFake(0,5,LEDcolor);
    TurnOnRowFake(0,1,LEDcolor);
    TurnOnColumnFake(0,5,LEDcolor);
    TurnOnColumnFake(0,1,LEDcolor);
  }
  else if((Xdegree >= 18*3-9) && Xdegree < (18*4-9)){
    
	TurnOnRowFake(3,1,LEDcolor);
    TurnOnRowFake(1,5,LEDcolor);
    TurnOnColumnFake(4,5,LEDcolor);
    TurnOnColumnFake(2,5,LEDcolor);
  }
  else if((Xdegree >= 18*4-9) && Xdegree < (18*5-9)){
    TurnOnRowFake(3,2,LEDcolor);
    TurnOnRowFake(1,4,LEDcolor);
    TurnOnColumnFake(4,4,LEDcolor);
    TurnOnColumnFake(2,4,LEDcolor);
  }
  else if((Xdegree >= 18*5-9) && Xdegree < (18*6-9)){
    TurnOnRowFake(3,3,LEDcolor);
    TurnOnRowFake(1,3,LEDcolor);
    TurnOnColumnFake(4,3,LEDcolor);
    TurnOnColumnFake(2,3,LEDcolor);
  }
  else if((Xdegree >= 18*6-9) && Xdegree < (18*7-9)){
    TurnOnRowFake(3,4,LEDcolor);
    TurnOnRowFake(1,2,LEDcolor);
    TurnOnColumnFake(4,2,LEDcolor);
    TurnOnColumnFake(2,2,LEDcolor);
  }
  else if((Xdegree >= 18*7-9) && Xdegree < (18*8-9)){
    TurnOnRowFake(3,5,LEDcolor);
    TurnOnRowFake(1,1,LEDcolor);
    TurnOnColumnFake(4,1,LEDcolor);
    TurnOnColumnFake(2,1,LEDcolor);
  }
  else if((Xdegree >= 18*8-9) && Xdegree < (18*9-9)){
    TurnOnRowFake(5,1,LEDcolor);
    TurnOnRowFake(5,5,LEDcolor);
    TurnOnColumnFake(5,1,LEDcolor);
    TurnOnColumnFake(5,5,LEDcolor);
  }
  else if((Xdegree >= 18*9-9) && Xdegree < (18*10-9)){
    AddOnLED(5,4,4,LEDcolor);
    AddOnLED(5,4,3,LEDcolor);
    AddOnLED(5,4,2,LEDcolor);
    AddOnLED(5,3,4,LEDcolor);
    AddOnLED(5,3,2,LEDcolor);
    AddOnLED(5,2,4,LEDcolor);
    AddOnLED(5,2,3,LEDcolor);
    AddOnLED(5,2,2,LEDcolor);
  }else{
    AddOnLED(5,3,3,LEDcolor);
  }
  //********************************************************************
  if (Ydegree < 18-9){
    AddOnLED(1,3,3,LEDcolor2);
  }
  else if((Ydegree >= 18-9) && (Ydegree < (18*2-9))){
    AddOnLED(1,4,4,LEDcolor2);
    AddOnLED(1,4,3,LEDcolor2);
    AddOnLED(1,4,2,LEDcolor2);
    AddOnLED(1,3,4,LEDcolor2);
    AddOnLED(1,3,2,LEDcolor2);
    AddOnLED(1,2,4,LEDcolor2);
    AddOnLED(1,2,3,LEDcolor2);
    AddOnLED(1,2,2,LEDcolor2);
  }
  else if((Ydegree >= 18*2-9) && Ydegree < (18*3-9)){
    TurnOnRowFake(1,5,LEDcolor2);
    TurnOnRowFake(1,1,LEDcolor2);
    TurnOnColumnFake(1,5,LEDcolor2);
    TurnOnColumnFake(1,1,LEDcolor2);
  }
  else if((Ydegree >= 18*3-9) && Ydegree < (18*4-9)){
    TurnOnColumnFake(5,5,LEDcolor2);
    TurnOnColumnFake(0,5,LEDcolor2);
    TurnOnRowFake(4,1,LEDcolor2);
    TurnOnRowFake(2,5,LEDcolor2);
  }
  else if((Ydegree >= 18*4-9) && Ydegree < (18*5-9)){
    TurnOnColumnFake(5,4,LEDcolor2);
    TurnOnColumnFake(0,4,LEDcolor2);
    TurnOnRowFake(4,2,LEDcolor2);
    TurnOnRowFake(2,4,LEDcolor2);
  }
  else if((Ydegree >= 18*5-9) && Ydegree < (18*6-9)){
    TurnOnColumnFake(5,3,LEDcolor2);
    TurnOnColumnFake(0,3,LEDcolor2);
    TurnOnRowFake(4,3,LEDcolor2);
    TurnOnRowFake(2,3,LEDcolor2);
  }
  else if((Ydegree >= 18*6-9) && Ydegree < (18*7-9)){
    TurnOnColumnFake(5,2,LEDcolor2);
    TurnOnColumnFake(0,2,LEDcolor2);
    TurnOnRowFake(4,4,LEDcolor2);
    TurnOnRowFake(2,2,LEDcolor2);
  }else if((Ydegree >= 18*7-9) && Ydegree < (18*8-9)){
    TurnOnColumnFake(5,1,LEDcolor2);
    TurnOnColumnFake(0,1,LEDcolor2);
    TurnOnRowFake(4,5,LEDcolor2);
    TurnOnRowFake(2,1,LEDcolor2);
  }else if((Ydegree >= 18*8-9) && Ydegree < (18*9-9)){
    TurnOnRowFake(3,1,LEDcolor2);
    TurnOnRowFake(3,5,LEDcolor2);
    TurnOnColumnFake(3,1,LEDcolor2);
    TurnOnColumnFake(3,5,LEDcolor2);
  }else if((Ydegree >= 18*9-9) && Ydegree < (18*10-9)){
    AddOnLED(3,4,4,LEDcolor2);
    AddOnLED(3,4,3,LEDcolor2);
    AddOnLED(3,4,2,LEDcolor2);
    AddOnLED(3,3,4,LEDcolor2);
    AddOnLED(3,3,2,LEDcolor2);
    AddOnLED(3,2,4,LEDcolor2);
    AddOnLED(3,2,3,LEDcolor2);
    AddOnLED(3,2,2,LEDcolor2);
  }else{
    AddOnLED(3,3,3,LEDcolor2);
  }
    //********************************************************************
  if (Zdegree < 18-9){
    AddOnLED(4,3,3,LEDcolor3);
  }else if((Zdegree >= 18-9) && (Zdegree < (18*2-9))){
    AddOnLED(4,4,4,LEDcolor3);
    AddOnLED(4,4,3,LEDcolor3);
    AddOnLED(4,4,2,LEDcolor3);
    AddOnLED(4,3,4,LEDcolor3);
    AddOnLED(4,3,2,LEDcolor3);
    AddOnLED(4,2,4,LEDcolor3);
    AddOnLED(4,2,3,LEDcolor3);
    AddOnLED(4,2,2,LEDcolor3);
  }else if((Zdegree >= 18*2-9) && Zdegree < (18*3-9)){
    TurnOnRowFake(4,5,LEDcolor3);
    TurnOnRowFake(4,1,LEDcolor3);
    TurnOnColumnFake(4,5,LEDcolor3);
    TurnOnColumnFake(4,1,LEDcolor3);
  }else if((Zdegree >= 18*3-9) && Zdegree < (18*4-9)){
    TurnOnRowFake(0,1,LEDcolor3);
    TurnOnRowFake(5,5,LEDcolor3);
    TurnOnColumnFake(1,1,LEDcolor3);
    TurnOnColumnFake(3,1,LEDcolor3);
  }else if((Zdegree >= 18*4-9) && Zdegree < (18*5-9)){
    TurnOnRowFake(0,2,LEDcolor3);
    TurnOnRowFake(5,4,LEDcolor3);
    TurnOnColumnFake(1,2,LEDcolor3);
    TurnOnColumnFake(3,2,LEDcolor3);
  }else if((Zdegree >= 18*5-9) && Zdegree < (18*6-9)){
    TurnOnRowFake(0,3,LEDcolor3);
    TurnOnRowFake(5,3,LEDcolor3);
    TurnOnColumnFake(1,3,LEDcolor3);
    TurnOnColumnFake(3,3,LEDcolor3);
  }else if((Zdegree >= 18*6-9) && Zdegree < (18*7-9)){
    TurnOnRowFake(0,4,LEDcolor3);
    TurnOnRowFake(5,2,LEDcolor3);
    TurnOnColumnFake(1,4,LEDcolor3);
    TurnOnColumnFake(3,4,LEDcolor3);
  }else if((Zdegree >= 18*7-9) && Zdegree < (18*8-9)){
    TurnOnRowFake(0,5,LEDcolor3);
    TurnOnRowFake(5,1,LEDcolor3);
    TurnOnColumnFake(1,5,LEDcolor3);
    TurnOnColumnFake(3,5,LEDcolor3);
  }else if((Zdegree >= 18*8-9) && Zdegree < (18*9-9)){
    TurnOnRowFake(2,1,LEDcolor3);
    TurnOnRowFake(2,5,LEDcolor3);
    TurnOnColumnFake(2,1,LEDcolor3);
    TurnOnColumnFake(2,5,LEDcolor3);
  }else if((Zdegree >= 18*9-9) && Zdegree < (18*10-9)){
    AddOnLED(2,4,4,LEDcolor3);
    AddOnLED(2,4,3,LEDcolor3);
    AddOnLED(2,4,2,LEDcolor3);
    AddOnLED(2,3,4,LEDcolor3);
    AddOnLED(2,3,2,LEDcolor3);
    AddOnLED(2,2,4,LEDcolor3);
    AddOnLED(2,2,3,LEDcolor3);
    AddOnLED(2,2,2,LEDcolor3);
  }else{
    AddOnLED(2,3,3,LEDcolor3);
  }
	boolean remember1;
	boolean remember2;
	boolean remember3;
	faceDISPLAY = 0;
	int i = 0;
	int columnPosition;
	int rowPosition;
	if(((Ydegree>45&&Ydegree<135)&&(Zdegree>45&&Zdegree<135)) && Xdegree<=55){  // 45 STRICTEST
		faceDISPLAY = 0;
	}else if(((Xdegree>45&&Xdegree<135)&&(Zdegree>45&&Zdegree<135)) && Ydegree<=55){
		faceDISPLAY = 1;
	}else if(((Ydegree>45&&Ydegree<135)&&(Xdegree>45&&Xdegree<135)) && Zdegree<=55){
		faceDISPLAY = 4;
	}else if(((Ydegree>45&&Ydegree<135)&&(Zdegree>45&&Zdegree<135)) && Xdegree>=125){ //135 
		faceDISPLAY = 5;
	}else if(((Xdegree>45&&Xdegree<135)&&(Zdegree>45&&Zdegree<135)) && Ydegree>=125){
		faceDISPLAY = 3;
	}else if(((Ydegree>45&&Ydegree<135)&&(Xdegree>45&&Xdegree<135)) && Zdegree>=125){
		faceDISPLAY = 2;
	}else{
		// Serial.println(" OOOOOPPPPSSSS!!!!!!!!! No hit.");
		if(((Ydegree>45&&Ydegree<135)||(Zdegree>45&&Zdegree<135)) && Xdegree<=45){  // 45 LESS STRICT
			faceDISPLAY = 0;
		}else if(((Xdegree>45&&Xdegree<135)||(Zdegree>45&&Zdegree<135)) && Ydegree<=45){
			faceDISPLAY = 1;
		}else if(((Ydegree>45&&Ydegree<135)||(Xdegree>45&&Xdegree<135)) && Zdegree<=45){
			faceDISPLAY = 4;
		}else if(((Ydegree>45&&Ydegree<135)||(Zdegree>45&&Zdegree<135)) && Xdegree>=135){ //135 
			faceDISPLAY = 5;
		}else if(((Xdegree>45&&Xdegree<135)||(Zdegree>45&&Zdegree<135)) && Ydegree>=135){
			faceDISPLAY = 3;
		}else if(((Ydegree>45&&Ydegree<135)||(Xdegree>45&&Xdegree<135)) && Zdegree>=135){
			faceDISPLAY = 2;
		}else{
			// Serial.println(" OOPSS********** No hit again.");
			if(((Ydegree>45&&Ydegree<135)||(Zdegree>45&&Zdegree<135)) && Xdegree<=50){  // 45 LEAST STRICT
				faceDISPLAY = 0;
			}else if(((Xdegree>45&&Xdegree<135)||(Zdegree>45&&Zdegree<135)) && Ydegree<=50){
				faceDISPLAY = 1;
			}else if(((Ydegree>45&&Ydegree<135)||(Xdegree>45&&Xdegree<135)) && Zdegree<=50){
				faceDISPLAY = 4;
			}else if(((Ydegree>45&&Ydegree<135)||(Zdegree>45&&Zdegree<135)) && Xdegree>=130){ //135 
				faceDISPLAY = 5;
			}else if(((Xdegree>45&&Xdegree<135)||(Zdegree>45&&Zdegree<135)) && Ydegree>=130){
				faceDISPLAY = 3;
			}else if(((Ydegree>45&&Ydegree<135)||(Xdegree>45&&Xdegree<135)) && Zdegree>=130){
				faceDISPLAY = 2;
			}else{			
				// Serial.println(" OOPSS***************** No hit again AGAIN.");
			}
		}
	}
	
	columnDISPLAY = 0;
	while(columnDISPLAY<5){
			//turn all LEDs off
		rowDISPLAY = 0;
		while(rowDISPLAY<14){
			rowDISPLAY++;
			remember1 = fakeLEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]; //blue
			rowDISPLAY++;
			remember2 = fakeLEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]; //green
			rowDISPLAY++;
			remember3 = fakeLEDs[faceDISPLAY][columnDISPLAY][rowDISPLAY]; //red
			if(faceDISPLAY==0 || faceDISPLAY==5){
				if(remember3&&remember2){  //red and green
					columnPosition = columnDISPLAY+1;
					rowPosition = (rowDISPLAY/3);
				}
			}else if(faceDISPLAY==4 || faceDISPLAY==2){
				if(remember1&&remember2){
					columnPosition = columnDISPLAY+1;
					rowPosition = (rowDISPLAY/3);
				}
			}else if(faceDISPLAY==1 || faceDISPLAY==3){
				if(remember1&&remember3){
					columnPosition = columnDISPLAY+1;
					rowPosition = (rowDISPLAY/3);
				}
			}
		}
		columnDISPLAY++;
	}
	
	if(columnPosition<1 || columnPosition>5 || rowPosition<1 || rowPosition>5){ //in this case keep previous value
		// Serial.print("Error!! Face is ");
		// Serial.print(faceDISPLAY);
		// Serial.print(", Column is ");
		// Serial.print(columnPosition);
		// Serial.print(",   Row is ");
		// Serial.print(rowPosition);
		// Serial.print(",              X degree is ");
		// Serial.print((int) Xdegree);
		// Serial.print(", Y degree is ");
		// Serial.print((int) Ydegree);
		// Serial.print(", Z degree is ");
		// Serial.print((int) Zdegree);
		// Serial.println(" ");
		if(columnPosition<1){
			columnPosition=1;
		}else if(columnPosition>5){
			columnPosition=5;
		}
		if(rowPosition<1){
			rowPosition=1;
		}else if(rowPosition>5){
			rowPosition=5;
		}
	}else{
		// Serial.print("Face is ");
		// Serial.print(faceDISPLAY);
		// Serial.print(", Column is ");
		// Serial.print(columnPosition);
		// Serial.print(",   Row is ");
		// Serial.print(rowPosition);
		// Serial.print(",               X degree is ");
		// Serial.print((int) Xdegree);
		// Serial.print(", Y degree is ");
		// Serial.print((int) Ydegree);
		// Serial.print(", Z degree is ");
		// Serial.print((int) Zdegree);
		// Serial.println(" ");
		location[0]=faceDISPLAY;
		location[1]=columnPosition;
		location[2]=rowPosition;
	}
	//TurnOnSingleLED(location[0],location[1],location[2], 0);
}

void transmitGameState(){
    payload[30] = 64 + GameState;
    payload[31] = 65; //ID for F1 = 65, F2 = 66, F3 = 67
    xbee.send(tx);
	//ShowTransmit(LEDs);
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
          while(i<(pauseBetweenNotes/20)){
            displayLEDs(LEDs);
            //delay(pauseBetweenNotes);
			transmitGameState();
            i++;
          }
          noTone(5);
        }
	//displayLEDs(LEDs);
}

void StartUp(){
  //for the future can do a starting up sequence
  int i = 0;
  while(i<100){
	ShowOff();
	displayLEDs(LEDs);
	i++;
  }
  //SOUND CODE
  //import from Conrad?
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
          while(i<(pauseBetweenNotes/9)){
            displayLEDs(LEDs);
			transmitGameState();
            //delay(pauseBetweenNotes);
            i++;
          }
          // stop the tone playing:
          noTone(5);
        }    
}

void SelectGame(){ //used after starting up for starting a game
	//updateaccel(x,y,z);
	StartUp();
	TurnOffCubeLED();
	TurnOnSingleLED(0,3,1,2);//top face show #1
	TurnOnSingleLED(0,3,2,2);
	TurnOnSingleLED(0,3,3,2);
	TurnOnSingleLED(0,3,4,2);
	TurnOnSingleLED(0,3,5,2);
	TurnOnSingleLED(4,1,2,2);//front face show #2
	TurnOnSingleLED(4,1,3,2);
	TurnOnSingleLED(4,1,4,2);
	TurnOnSingleLED(4,2,4,2);
	TurnOnSingleLED(4,3,2,2);
	TurnOnSingleLED(4,3,3,2);
	TurnOnSingleLED(4,3,4,2);
	TurnOnSingleLED(4,4,2,2);
	TurnOnSingleLED(4,5,4,2);
	TurnOnSingleLED(4,5,3,2);
	TurnOnSingleLED(4,5,2,2);
	TurnOnSingleLED(3,2,1,2);//front face show #3
	TurnOnSingleLED(3,3,1,2);
	TurnOnSingleLED(3,4,1,2);
	TurnOnSingleLED(3,2,2,2);
	TurnOnSingleLED(3,4,3,2);
	TurnOnSingleLED(3,3,3,2);
	TurnOnSingleLED(3,2,3,2);
	TurnOnSingleLED(3,2,4,2);
	TurnOnSingleLED(3,4,5,2);
	TurnOnSingleLED(3,3,5,2);
	TurnOnSingleLED(3,2,5,2);
	int GameSelected = 6;
	int i = 0;
	int Testaccel = acceljerk(x,y,z);
	
	displayLEDs(LEDs);
	while(Testaccel==0){
		Testaccel = acceljerk(x,y,z);
		displayLEDs(LEDs);
		
		if(GameState==-1){
			return;
		}
	}
	//change to green!
	TurnOnSingleLED(0,3,1,1);//top face show #1
	TurnOnSingleLED(0,3,2,1);
	TurnOnSingleLED(0,3,3,1);
	TurnOnSingleLED(0,3,4,1);
	TurnOnSingleLED(0,3,5,1);
	TurnOnSingleLED(4,1,2,1);//front face show #2
	TurnOnSingleLED(4,1,3,1);
	TurnOnSingleLED(4,1,4,1);
	TurnOnSingleLED(4,2,4,1);
	TurnOnSingleLED(4,3,2,1);
	TurnOnSingleLED(4,3,3,1);
	TurnOnSingleLED(4,3,4,1);
	TurnOnSingleLED(4,4,2,1);
	TurnOnSingleLED(4,5,4,1);
	TurnOnSingleLED(4,5,3,1);
	TurnOnSingleLED(4,5,2,1);
	TurnOnSingleLED(3,2,1,1);//front face show #3
	TurnOnSingleLED(3,3,1,1);
	TurnOnSingleLED(3,4,1,1);
	TurnOnSingleLED(3,2,2,1);
	TurnOnSingleLED(3,4,3,1);
	TurnOnSingleLED(3,3,3,1);
	TurnOnSingleLED(3,2,3,1);
	TurnOnSingleLED(3,2,4,1);
	TurnOnSingleLED(3,4,5,1);
	TurnOnSingleLED(3,3,5,1);
	TurnOnSingleLED(3,2,5,1);
	
	i = 0;
	while(i<20){
		displayLEDs(LEDs);
		i++;
	}
	i = 0;
	int top = 0;
	int four = 0;
	int three = 0;
	int accelResult;
	while(i<100){
		accelResult = acceltop(x,y,z);
		if(accelResult == 0){
			top = top++;
		}
		if(accelResult == 4){
			four = four++;
		}
		if(accelResult == 3){
			three = three++;
		}
		i++;
		displayLEDs(LEDs);
	}
	
	if(top>four && top>three){
		GameState = 10;
	}else if(four>top && four>three){
		GameState = 20;
	}else{
		GameState = 30;
	}
	displayLEDs(LEDs);
}

void StartGame3(){
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

void StartGame2(){
	updateaccel(x,y,z);
        int i = 0;
        int melody[] = {NOTE_F4,NOTE_D4, NOTE_C5};
        // note durations: 4 = quarter note, 8 = eighth note, etc.:
        int noteDurations[] = {8, 8, 8};
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

void StartGame1(){ //navigate your blue guy across the cube, don't hit the rails, eat all the apples
	int melody[] = {
	  NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_B4, NOTE_A4, NOTE_G3, NOTE_F3,NOTE_E3,NOTE_D3,NOTE_C3, NOTE_B3, NOTE_A3, NOTE_G2,NOTE_F2,NOTE_E2};
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = {
	  8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8, };	
	int i = 0;

	TurnOnSingleLED(0,1,2,0);//turn on red rails/maze: Should be 61 of them:
	
	// TurnOnSingleLED(4,3,3,1);//apple in the middle of front face
	// TurnOnSingleLED(5,3,3,1);//apple in middle of bottom face
	// TurnOnSingleLED(2,3,3,1);//apple in middle of back face
	// TurnOnSingleLED(0,3,3,2);//Your starting spot, you're blue
	
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

void StartGameMusic(){
	updateaccel(x,y,z);
	int i = 0;
	int melody[] = {NOTE_F4,NOTE_D4, NOTE_C5};  
	int noteDurations[] = {8, 8, 8}; // note durations: 4 = quarter note, 8 = eighth note, etc.
	for (int thisNote = 0; thisNote < 3; thisNote++) {
		// to calculate the note duration, take one second divided by the note type. e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000/noteDurations[thisNote];
		tone(5, melody[thisNote],noteDuration);

		// to distinguish the notes, set a minimum time between them. The note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		i=0;
		while(i<(pauseBetweenNotes/7)){
			displayLEDs(LEDs);
			i++;
		}
		noTone(5);
	}
}

void Game4(){ //run away from blocks
	StartGameMusic();
	TurnOffCubeLED();
	//...
	int i;
	int GamePace = 35;
	int movePitchB1 = 0;
	int moveYawB1 = 1;
	int moveRollB1 = 0;
	int movePitchB2 = 0;
	int moveYawB2 = 0;
	int moveRollB2 = 1;
	int player[3];
	player[0] = 3; //face
	player[1] = 3; //column
	player[2] = 3; //row
	int block1[3];
	block1[0] = 4; //face
	block1[1] = 3; //column
	block1[2] = 3; //row
	int block2[3];
	block2[0] = 0; //face
	block2[1] = 3; //column
	block2[2] = 3; //row
	int PrevBlockFace1 = block1[0];
	int PrevBlockFace2 = block2[0];
	
	while(1){
		if(PrevBlockFace1 != block1[0]){
			i = 1;
		}else{
			i = 0;
		}
		PrevBlockFace1 = block1[0];
		if(block1[0]==0 || block1[0]==5 || i==1){
			if((int) random(5) == 1){
				movePitchB1 = random(3) - 1;
				moveRollB1 = random(3) - 1;
			}
		}else if(block1[0]==2 || block1[0]==4){
			if((int) random(5) == 1){
				movePitchB1 = random(3) - 1;
				moveYawB1 = random(3) - 1;
			}
		}else{
			if((int) random(5) == 1){
				moveYawB1 = random(3) - 1;
				moveRollB1 = random(3) - 1;
			}
		}
		
		if(PrevBlockFace1 != block1[0]){
			i = 1;
		}else{
			i = 0;
		}
		PrevBlockFace2 = block2[0];
		if(block2[0]==0 || block2[0]==5 || i==1){
			if((int) random(5) == 1){
				movePitchB2 = random(3) - 1;
				moveRollB2 = random(3) - 1;
			}
		}else if(block2[0]==2 || block2[0]==4){
			if((int) random(5) == 1){
				movePitchB2 = random(3) - 1;
				moveYawB2 = random(3) - 1;
			}
		}else{
			if((int) random(5) == 1){
				moveYawB2 = random(3) - 1;
				moveRollB2 = random(3) - 1;
			}
		}
		MoveBlock(block1, movePitchB1, moveYawB1, moveRollB1, 0);
		MoveBlock(block2, movePitchB2, moveYawB2, moveRollB2, 0);
		displayLEDs(LEDs);
		TurnOffSingleLED(player[0],player[1],player[2]);
		topPosition(player);
		TurnOnSingleLED(player[0],player[1],player[2],1);
		
		i = 0;
		while(i<GamePace){
			displayLEDs(LEDs);
			TurnOffSingleLED(player[0],player[1],player[2]);
			topPosition(player);
			TurnOnSingleLED(player[0],player[1],player[2],1);
			i++;
		}
	}
	
	
	
}

void Game3(){ //top face game
	updateaccel(x,y,z);
	int i = 0;
	StartGame3();
	i = random(6);
	while(i == acceltop(x,y,z)){
	  i = random(6);
	}
	TurnOffCubeLED();
	
	TurnOnColumn(i,1,2);
	TurnOnColumn(i,2,2);
	TurnOnColumn(i,3,2);
	TurnOnColumn(i,4,2);
	TurnOnColumn(i,5,2);
	while(acceltop(x,y,z)!=i){
	  displayLEDs(LEDs);
		if(GameState==-1){
			return;
		}
	}
	WinGame();
	GameState = 0;
} 

void Game2(){ //multi player reflexes game
	digitalWrite(XbeeSleep, 0); //wake up Xbee since it will be used now
	updateaccel(x,y,z);
	int i = 0;
	GameState = 20;
	StartGame2();
	transmitGameState();
	receivedGameState = 0;
	while(!(receivedGameState==20 || receivedGameState==25)){ //wait until a partner is doing Game2
		displayLEDs(LEDs);
		xbee.readPacket(); //this method has less delay than (xbee.readPacket(1)
		
		if (xbee.getResponse().isAvailable()) {  // got something
			xbee.getResponse().getRx16Response(rx16);
			Serial.print(" got somethin: ");
			receivedGameState = (int) (rx16.getData(30)&31); //where the color ID is, can be used for GameState too (why not?)
			Serial.println(receivedGameState);
			displayLEDs(LEDs);
		}
		transmitGameState();
		
		if(GameState==-1){
			digitalWrite(XbeeSleep, 1);
			return;
		}
	}
	Serial.println("i found a friend");
    displayLEDs(LEDs);
	transmitGameState();
	FriendedTone(); //show that a friend was found and the game can start now
	i = random(600)+100;
	while(i>0){ // wait for a random amount of time
		displayLEDs(LEDs);
		transmitGameState();
		i--;
		if(GameState==-1){
			digitalWrite(XbeeSleep, 1);
			return;
		}
	}
	Serial.println(" ready to ShowOff ");
	GameState = 25; //means that it's ready to ShowOff
	transmitGameState(); //tell the friend that it's ready to ShowOff
	i = 0;
	while(!(receivedGameState==25 || receivedGameState==27)){ //wait until the friend is ready to ShowOff too
		displayLEDs(LEDs);
		xbee.readPacket();
		
		if (xbee.getResponse().isAvailable()) {  // got something
			xbee.getResponse().getRx16Response(rx16);
			receivedGameState = (int) (rx16.getData(30)&31); //where the color ID is, can be used for GameState too (why not?)
			displayLEDs(LEDs);
			Serial.print(" got somethin after friended: ");
			Serial.println(receivedGameState);
		}
		transmitGameState();
		
		if(GameState==-1){
			digitalWrite(XbeeSleep, 1);
			return;
		}
	}
	if(receivedGameState!=27){
		GameState = 27;//means that it's ready to ShowOff and it has a head start
		i = 0;
		Serial.println("head start begin");
		while(i<100){
			displayLEDs(LEDs);
			transmitGameState();
			i++;
		 }
		Serial.println("head start end");
	}

	while(receivedGameState!=29 && acceljerk(x,y,z)==0){ //no one has won yet
		ShowOff();
		displayLEDs(LEDs);
		xbee.readPacket();
		if (xbee.getResponse().isAvailable()) {  // got something
			xbee.getResponse().getRx16Response(rx16);
			receivedGameState = (int) (rx16.getData(30)&31); //where the color ID is, can be used for GameState too (why not?)
			displayLEDs(LEDs);
		}
		
		if(GameState==-1){
			digitalWrite(XbeeSleep, 1);
			return;
		}
		
		GameState = 27;
		transmitGameState();
	}
	
	if(receivedGameState == 29){
		LoseGame();
		Serial.println("LOSE");
	}else{
		GameState = 29;
		i = 0;
		while(i<25){
			displayLEDs(LEDs);
			transmitGameState();
			i++;
		}
		WinGame();
		i = 0;
		while(i<25){
			displayLEDs(LEDs);
			transmitGameState();
			i++;
		}
		Serial.println("WIN");
	}
	
	if(GameState==-1){
		digitalWrite(XbeeSleep, 1);
		return;
	}
	
	GameState = 0; //go back to game selection screen
	digitalWrite(XbeeSleep, 1); //put xbee back to sleep to save battery
}

void Game1(){ //maze game
	updateaccel(x,y,z);
	StartGame1();
	int solvedFaces[] = {6,6,6,6,6};
	int i = 0;
        Serial.println("game 1");
	while(GameState<18 && GameState>9){ //GameState=18 for Lose, GameState=19 for Win
		//Serial.println(GameState);
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
		GameState = 0;
	}else if(GameState==19){
		WinGame();
		GameState = 0;
	}
	//GameState = 0;
}

int idle(){
	int i = 0;
	digitalWrite(XbeeSleep, 1);
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
	displayLEDs(LEDs);
	if(acceljerk(x,y,z)==1){
		return 0;
	}else{
		return -1;
	}
	//displayLEDs(LEDs);
}

void TestFaces(){
	int i = 0;
	while(i<40){
		TurnOnCubeLED(0);
		displayLEDs(LEDs);
		i++;
	}
	while(i<80){
		TurnOnCubeLED(1);
		displayLEDs(LEDs);
		i++;
	}
	while(i<120){
		TurnOnCubeLED(2);
		displayLEDs(LEDs);
		i++;
	}
	i=0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
	// int color = 0;
	// int location[3];
	// location[0] = 3; //face
	// location[1] = 4; //column
	// location[2] = 4; //row
	// while(1){
		// displayLEDs(LEDs);
		// topPosition(location);
		// TurnOnSingleLED(location[0],location[1],location[2], color);
	// }
	// Game4();
	
	while(GameState == -1){   
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
	
	// Serial.println("hey");
	
//For testing MoveLED():
		// int location[3];
		// location[0] = 3; //face
		// location[1] = 4; //column
		// location[2] = 4; //row
		// TurnOnSingleLED(location[0], location[1], location[2], 2);
		// int temploc;
		// int i = 0;
		// int rando1;
		// int rando2;
		// int rando3;
		// while(i<40){
			// displayLEDs(LEDs);
			// i++;
		// }
		// while(1){
			// rando1 = (int) random(2);
			// rando2 = (int) random(2);
			// rando3 = (int) random(2);
			// MoveBlock(location, 0, 1, 0, 2);
			// i = 0;
			// while(i<180){
				// displayLEDs(LEDs);
				// i++;
			// }
		// }
}
