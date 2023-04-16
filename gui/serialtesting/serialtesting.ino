unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds

const int MAX_STRING_SIZE = 64;
char inputString[MAX_STRING_SIZE];
int stringIndex = 0;

void setup() {
  Serial.begin(9600); // initialize serial communication at 9600 bits per second
  startMillis = millis();  //initial start time
}

void loop() {
  if (Serial.available() > 0) { // if data is available to read
    String test = Serial.readString();
    // char inByte = Serial.read(); // read a byte
    // if (inByte == '\n') { // if newline character is received
    //   inputString[stringIndex] = '\0'; // terminate the string
    //   char* firstValue = strtok(inputString, ","); // split the string based on the comma delimiter
    //   char* secondValue = strtok(NULL, ",");
    //   Serial.print("First value: ");
    //   Serial.println(firstValue);
    //   Serial.print("Second value: ");
    //   Serial.println(secondValue);
    //   stringIndex = 0; // reset the index
    // } else {
    //   inputString[stringIndex] = inByte; // store the byte in the character array
    //   stringIndex++; // increment the index
    // }
    Serial.print(test);
  }

  // currentMillis = millis();

  // if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  // {
  //   // Testing reading from the arduino
  //   int value = random(10); // Generate a random value between 0 and 10

  //   // Send the random value through serial communication
  //   Serial.print("Torque: ");
  //   Serial.println(value);
  //   startMillis = currentMillis;  //IMPORTANT to save the start time
  // }
}