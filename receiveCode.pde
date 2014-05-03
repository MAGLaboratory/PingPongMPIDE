
char receive_function(char* buff,unsigned char sizevar) 
{ 
  static char ctr = 0; // store the current position in the buff array 
  char ch; // store the last character received 
  if (Serial.available() > 0) // true when characters in serial input buffer 
  { 
    ch = Serial.read(); // store character from buffer in ch 
    if( ctr < sizevar) { // if the ctr is less than your buffer yet 
      buff[ctr++] = ch; // add it to your buffer 
    } 
    if (ch == '\r') // if that character is a carriage return 
    { 
      buff[ctr-1] = 0; // replace '\r' with '\0' (string termination) 
      ctr = 0; // reset the pointer 
      Serial.print("Command: "); // print a string and stay on the same line 
      if(buff[0] != 0) 
        Serial.println(buff); // print received followed by a new line 
      else 
        Serial.print("\n"); 
      return 1; 
    } 
    else 
      return 0; 
  } //end serial was available 
  return 0; 
} 

void Reset() 
{ 
  //VIRTUAL_PROGRAM_BUTTON_TRIS = 0; //Set virtual button as output 
  //VIRTUAL_PROGRAM_BUTTON = 1; //push virtual button 
  SYSKEY = 0x00000000; //write invalid key to force lock 
  SYSKEY = 0xAA996655; //write key1 to SYSKEY 
  SYSKEY = 0x556699AA; //write key2 to SYSKEY // OSCCON is now unlocked 
  RSWRSTSET = 1; //set SWRST bit to arm reset 
  unsigned int dummy; 
  dummy = RSWRST; //read RSWRST register to trigger reset 
  while(1); //prevent any unwanted code execution until reset occurs 
} 



