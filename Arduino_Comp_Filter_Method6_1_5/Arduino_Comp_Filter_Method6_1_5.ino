#include<Wire.h>
// Prepare MPU for use
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

// Set Integers for Auto-Calibration 
int AcX_offset,AcY_offset,AcZ_offset,GyX_offset,GyY_offset,GyZ_offset;
int mean_AcX,mean_AcY,mean_AcZ,mean_GyX,mean_GyY,mean_GyZ;
float mean_Tmp;

// Set Relative Error Parameters for Calibration
int buffersize = 200;

// Initialize Tilt variables
float angle_AcY, angle_GyZ, filtered_angle_Y, filtered_angle_Z;
float filtered_angle=0, smoothed_angle=0, total_angle=0;
float angle_offset;
float alpha = 0.98;
float time, t_now, dt;

// Initialize Running Average Variables
int numReadings = 3;
float readings[3];
int readIndex = 0;

// Initialize Peak Determing Variables
int stepCount, index, lastIndex, minIndex, lastMinIndex;
float peak = 0, maximumPeak = 0;

// Initialize Gait Baseline Variables
float initial_time=0, final_time=0;
float total_time=0;
float bpm = 0;
float peakArray[10];
float minArray[10];

// Initialize Metronome Variables
int speaker = 8;
int metronome_BPM = 0;
int metronome_pulse = 0;
int timeOfLastTone = 0;
int currentTime = 0;

// Initialize Freezing Variables
float threshold=0.3, upperBound=0;
double timeSinceLastStep = 0; // time since person took a step
double timeSinceAcY = 0; // time since person tried to move foot
int consecutiveAnglesAroundZero = 0; // consecutive angles around 0; indicates stopping instead of freezing
double changeInTimeForFreezing = 0;
int freezeCount = 0;
int moveCount = 0;
int stillCount = 0;

// Initialize Movement Type Variables
float upper_threshold = 0, lower_threshold = 0;
float max_threshold = 0, min_threshold = 0;
float maxMinValue = 0;

void setup(){
  // Turn on MPU
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  // Turn on Serial Monitor
  Serial.begin(115200);
  while (!Serial){
  }
  // Set initial Time
  time = millis();
  
  // Prepare MPU for Use
  initialize_functions();
  //Serial.println(upper_threshold);
  Serial.println("Annnnnnnnd I'm spent");
  delay(200);
  
  // Make an Array of Zeros for Running Average
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
  changeInTimeForFreezing = millis();
}

int slow = 0;

void loop() {
  
  
  // Begin Communication with MPU
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    
    // Read Data Values
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();
    AcZ = Wire.read()<<8|Wire.read();
    Tmp = Wire.read()<<8|Wire.read();
    GyX = Wire.read()<<8|Wire.read(); 
    GyY = Wire.read()<<8|Wire.read();
    GyZ = Wire.read()<<8|Wire.read();
    
    // Take the Current Time
    unsigned long t_now = millis();
    
    // Convert Gyro Values to Degrees/sec
    float FS_SEL = 131;
    GyZ = (GyZ - GyZ_offset)/FS_SEL;
    
    // Calculate the Angle from Accelerometer Data
    angle_AcY = 2*atan(AcY / sqrt(pow(AcX,2) + pow(AcZ,2)));
    
    // Calculate Sampling Interval
    dt = (t_now - time)/1000.0;
    
    // Calculate the Angle from Gyro Data
    angle_GyZ = GyZ*dt;
    
    // Apply Complimentary Filter to Fuse Angle Data
    filtered_angle_Z = alpha*angle_GyZ + (1.0-alpha)*angle_AcY;
    
    // Apply Running Average of Angle Data
    total_angle = total_angle - readings[readIndex];
    readings[readIndex] = filtered_angle_Z;
    total_angle = total_angle + readings[readIndex];
    readIndex++;
    
    if (readIndex >= numReadings) {
      readIndex = 0;
    }
    
    filtered_angle = (total_angle/numReadings);
    
    // Smooth the Angle Data
    smoothed_angle = filtered_angle_Z - filtered_angle;
    
    // Update Time for Next Sample
    time = millis();
    
    // Zero the Smoothed Angle Data
    if (index == 10) {
      angle_offset = 0 - smoothed_angle;
    }
    
    smoothed_angle = smoothed_angle + angle_offset;
    
    if(isFreezing(smoothed_angle - 0.05, (millis()/1000.0), changeInTimeForFreezing, upper_threshold, 0)){
      changeInTimeForFreezing = millis()/1000.0;
      if(slow%10 == 0){
        freezeCount++;
        if(freezeCount > 12){
          moveCount = 0;
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.print("  FREEZE");
          Serial.print(", ");
          
          if(freezeCount == 13){            
            tone(speaker, 1024);
            timeOfLastTone = millis();
          }// play the first beat upon detection of freezing
          
        }// will turn on LED when past 7 or more sample evaluations return freezing
        
        if((freezeCount > 12 && currentTime - timeOfLastTone > (((1.0/metronome_BPM)*60000)))){
            tone(speaker, 1024);
            timeOfLastTone = millis();
        }// if person is still freezing, play more beats
        
        Serial.print(timeSinceLastStep);
        Serial.print(", angle = ");
        Serial.print(smoothed_angle);
        Serial.print(", AcY = ");
        Serial.print(AcY);
        Serial.print(", ");
        Serial.print(consecutiveAnglesAroundZero);
        Serial.print(", ");
        Serial.print(AcX);
        Serial.print("\n");
       
      }    
    }
    else{
      changeInTimeForFreezing = millis()/1000.0;
      if(slow%10 == 0){
        moveCount++;
        if(moveCount > 20){
          noTone(speaker);
          freezeCount = 0;
          //digitalWrite(LED_BUILTIN, LOW);
          Serial.print("NO FREEZE");
          Serial.print(", ");
        }// will turn off LED when past 10 or more sample evaluations return no freezing                  
        
        if(freezeCount > 0 && currentTime - timeOfLastTone > (((1.0/metronome_BPM)*60000))){
          tone(speaker, 1024);
          timeOfLastTone = millis();
        }
        Serial.print(timeSinceLastStep);
        Serial.print(", angle = ");
        Serial.print(smoothed_angle);
        Serial.print(", AcY = ");
        Serial.print(AcY);
        Serial.print(", ");
        Serial.print(consecutiveAnglesAroundZero);
        Serial.print(", ");
        Serial.print(AcX);
        Serial.print("\n");
      }     
    }
    
    currentTime = millis();
    
    if(currentTime - timeOfLastTone > 50){
      noTone(speaker);
    }// turn off speaker after 50 milliseconds
    
    // Increment Index
    index++;
    slow++;
    delay(5);
   
}

// Calculate all Necessary Initial Values
void initialize_functions() {
  Serial.println("\nPreparing sensors for use...");
  findMean_sensors();
  delay(200);
  Serial.println("\nCalibrating MPU...");
  calibrate_sensors();
  Serial.println("\nMPU Calibrated. Please take 5 steps to establish gait...");
  gait_baseline();
  metronome_rate();
  upperBound = get_upperBound(peakArray, 10);
  Serial.print("\nGait Established. SPM = ");
  Serial.print(bpm);
  Serial.print("\nCalculated Metronome = ");
  Serial.print(metronome_BPM);
  Serial.println("");
  upper_threshold = (max_threshold + upperBound)/2;
  delay(5000);
  Serial.println("\nPlease turn around to establish turning threshold...");
  turning_baseline();
  Serial.println("\nThreshold established");
  lower_threshold = min_threshold;
  Serial.println("\nCalculated upper threshold = ");
  Serial.println(upper_threshold);
  Serial.println("\nCalculated lower threshold = ");
  Serial.println(lower_threshold);
  Serial.println("\n");
}

// Calculate Mean data value from sensors 
void findMean_sensors() {
  
  // Initialize Necessary Variables
  long i=0,total_AcX=0,total_AcY=0,total_AcZ=0,total_Tmp=0, total_GyX=0,total_GyY=0,total_GyZ=0;
  
  // Loop Through the First 201 Data Points
  while (i<(buffersize+201)){
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();
    AcZ = Wire.read()<<8|Wire.read();
    Tmp = Wire.read()<<8|Wire.read();
    GyX = Wire.read()<<8|Wire.read(); 
    GyY = Wire.read()<<8|Wire.read();
    GyZ = Wire.read()<<8|Wire.read();
    
    // Dispose of First 20 Readings; Total the Next 200 Readings
    if (i > 20 && i <= (buffersize+20)) {
      total_AcX = total_AcX + AcX;
      total_AcY = total_AcY + AcY;
      total_AcZ = total_AcZ + AcZ;
      total_Tmp = total_Tmp + Tmp;
      total_GyX = total_GyX + GyX;
      total_GyY = total_GyY + GyY;
      total_GyZ = total_GyZ + GyZ;
    }
    
    // Average the 200 Totaled Values
    if (i==(buffersize + 20)){
      mean_AcX = (total_AcX/buffersize) + AcX_offset;
      mean_AcY = (total_AcY/buffersize) + AcY_offset;
      mean_AcZ = (total_AcZ/buffersize) + AcZ_offset;
      mean_GyX = (total_GyX/buffersize) + GyX_offset;
      mean_GyY = (total_GyY/buffersize) + GyY_offset;
      mean_GyZ = (total_GyZ/buffersize) + GyZ_offset;
      // Temperature equation from data sheet
      mean_Tmp = (total_Tmp/buffersize)/340.00+36.53;
    }
    i++;
    delay(5); 
  }
}

// Calculate the necessary offset to "zero" the readings
void calibrate_sensors() {
  findMean_sensors();
  
  // Calculate Difference Between Data Mean and "zero" position
  AcX_offset = 0 - mean_AcX;
  AcY_offset = 0 - mean_AcY;
  AcZ_offset = 16384 - mean_AcZ; 
  GyX_offset = 0 - mean_GyX;
  GyY_offset = 0 - mean_GyY;
  GyZ_offset = 0 - mean_GyZ;
}

// Determines the Patients Normal Walking Rate
void gait_baseline() {
  int p = 0;
  
  while (stepCount < 5) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();
    AcZ = Wire.read()<<8|Wire.read();
    Tmp = Wire.read()<<8|Wire.read();
    GyX = Wire.read()<<8|Wire.read(); 
    GyY = Wire.read()<<8|Wire.read();
    GyZ = Wire.read()<<8|Wire.read();
    
    unsigned long t_now = millis();
    
    float FS_SEL = 131;
    GyZ = (GyZ - GyZ_offset)/FS_SEL;
    
    angle_AcY = 2*atan(AcY / sqrt(pow(AcX,2) + pow(AcZ,2)));
    
    dt = (t_now - time)/1000.0;
    
    angle_GyZ = GyZ*dt;
    
    filtered_angle_Z = alpha*angle_GyZ + (1.0-alpha)*angle_AcY;
    
    total_angle = total_angle - readings[readIndex];
    readings[readIndex] = filtered_angle_Z;
    total_angle = total_angle + readings[readIndex];
    readIndex++;
    
    if (readIndex >= numReadings) {
      readIndex = 0;
    }
    
    filtered_angle = (total_angle/numReadings);
    
    smoothed_angle = filtered_angle_Z - filtered_angle;
    
    // Calculate Peak Values
    if ((smoothed_angle > 0.6) && (index-lastIndex >= 40)) {
      if (initial_time == 0) {
        initial_time = millis();
      }
      peakArray[p] = smoothed_angle;
      p++;
      lastIndex = index;
      stepCount++;
      Serial.println(stepCount);
    }
    
    // Update Time for Next Sample
    time = millis();
    
    // Increment Index
    index++;
    delay(5);
  }
  
  // Determine Total Time Required to Take 10 Steps
  final_time = millis();
  total_time = ((final_time - initial_time)/1000)/60;
  
  // Determine BPM
  bpm = stepCount/total_time;
  
  // Reset Index and lastIndex for Main Loop
  index = 0;
  lastIndex = 0;
  
  // Calculate Average Maximum Threshold
  for (int i = 0; i < 5; i++) {
    max_threshold = max_threshold + peakArray[i];
  }
  
  max_threshold = max_threshold / 5; 
}

// Determines the Patients Turning Data
void turning_baseline() {
  int turn_count = 0;
  int k = 0;
  
  while (turn_count < 4) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();
    AcZ = Wire.read()<<8|Wire.read();
    Tmp = Wire.read()<<8|Wire.read();
    GyX = Wire.read()<<8|Wire.read(); 
    GyY = Wire.read()<<8|Wire.read();
    GyZ = Wire.read()<<8|Wire.read();
    
    unsigned long t_now = millis();
    
    float FS_SEL = 131;
    GyZ = (GyZ - GyZ_offset)/FS_SEL;
    
    angle_AcY = 2*atan(AcY / sqrt(pow(AcX,2) + pow(AcZ,2)));
    
    dt = (t_now - time)/1000.0;
    
    angle_GyZ = GyZ*dt;
    
    filtered_angle_Z = alpha*angle_GyZ + (1.0-alpha)*angle_AcY;
    
    total_angle = total_angle - readings[readIndex];
    readings[readIndex] = filtered_angle_Z;
    total_angle = total_angle + readings[readIndex];
    readIndex++;
    
    if (readIndex >= numReadings) {
      readIndex = 0;
    }
    
    filtered_angle = (total_angle/numReadings);
    
    smoothed_angle = filtered_angle_Z - filtered_angle;
    
    // Find Peak of Turning Values
    if ((smoothed_angle < 0.4) && (smoothed_angle > 0.1) && (minIndex-lastMinIndex >= 40)) {
      minArray[k] = smoothed_angle;
      k++;
      lastMinIndex = minIndex;
      turn_count++;
    }
    
    time = millis();
    
    minIndex++;
    delay(5);
  }
  
  // Calculate Maximum of the Turning Data
  for (int i = 0; i < 10; i++) {
    if (minArray[i] > min_threshold) {
      min_threshold = minArray[i];
    }
  }
}

// Calculate the Rate for the Metronome --> Working (like another boss)
void metronome_rate() {
  // Add 0.5 for rounding
  metronome_BPM = bpm + (bpm*0.1)+0.5;
  
  // Determine Length of Time Between Beats
  metronome_pulse = (total_time/metronome_BPM)*60000;
}

// Calculate Minimum Upper Bound of Gait
float get_upperBound(float* array, int size_array) {
  float minimum = array[0];
  for (int i = 0; i < size_array; i++) {
    if (array[i] < minimum) {
      minimum = array[i];
    }
  }
  return minimum;
}

bool isFreezing(double pitchAngle, double timeNow, double lastTime, double thresholdUpper, double thresholdAcY){
  if(thresholdUpper < 0.85){
    thresholdUpper = 0.85;
  }
  
  timeSinceLastStep = timeSinceLastStep + timeNow - lastTime;
  timeSinceAcY = timeSinceAcY + timeNow - lastTime;
  
  /*double threshold = sqrt(pow(AcX, 2) + pow(AcY, 2) + pow(AcZ, 2));
  /int bound = 30000;
  */
  if(abs(AcY) > 7000 || AcY > 6000)
  {
    timeSinceAcY = 0;
  }// reset time since person tried to move foot
  else if(AcX < -20000){
    consecutiveAnglesAroundZero = 0;
  }
  
  if(pitchAngle > thresholdUpper - 0.05)
  {
    timeSinceLastStep = 0;
    consecutiveAnglesAroundZero = 0;    
    return false;
  }// if person took a step, reset timeSinceLastStep and timeSinceAcY
  
  if(pitchAngle < lower_threshold + 0.05){
    consecutiveAnglesAroundZero++;
  }
  else
  {
    consecutiveAnglesAroundZero = 0;
  }
    
  if(bpm > 80){    
    if(timeSinceLastStep > ((2/bpm)*60) && timeSinceAcY < ((2.5/bpm)*60) && consecutiveAnglesAroundZero < 105){ 
      return true;
    }// if timeSinceLastStep > period && timeSinceLastMovement < period, then return freezing
    else{
      return false;
    }
  }
  else if(bpm > 60 && bpm < 80){
    if(timeSinceLastStep > ((1.75/bpm)*60) && timeSinceAcY < ((2/bpm)*60) && consecutiveAnglesAroundZero < 125){ 
      return true;
    }// if timeSinceLastStep > period && timeSinceLastMovement < period, then return freezing
    else{
      return false;
    }
  }
  else{
    if(timeSinceLastStep > ((1.5/bpm)*60) && timeSinceAcY < ((1.5/bpm)*60) && consecutiveAnglesAroundZero < 145){ 
      return true;
    }// if timeSinceLastStep > period && timeSinceLastMovement < period, then return freezing
    else{
      return false;
    }
  }
  
  lastTime = millis()/1000.0;
}

/*
// Play the Metronome
void metronome() {
  int metronome_count = 0;
  
  // Output to the Metronome
  while (metronome_count < 5) {
    tone(speaker, 1024);
    delay(50);
    noTone(speaker);
    // Uses the Baseline + 10% Beat Interval Time for Delay
    metronome_count++;
    if(metronome_count == 5){
      break;
    }
    delay(((1.0/metronome_BPM)*60000)-50);
    
  }
}
*/
