// Setup Parameters
#include <Wire.h>
//#include <MatrixMath.h>
#include <math.h>
#include <DueTimer.h>
#include <Sweep.h>
int epsilon = 10; //DBSCAN cluster diameter
int MinPts = 2; //DBSCAN cluster min size 

// Create a Sweep device using Serial #1 (RX1 & TX1)
Sweep device(Serial1);
// Scan packet struct, used to store info for a single reading


// keeps track of how many scans have been collected
int scanCount = 0;
// keeps track of how many samples have been collected
int sampleCount = 0;

// Arrays to store attributes of collected scans
bool syncValues[1000];         // 1 -> first reading of new scan, 0 otherwise
float angles[1000];            // in degrees (accurate to the millidegree)
int distances[1000];      // in cm
int signalStrengths[1000]; // 0:255, higher is better

// define communication wires with iRobot
const int rxPin = 10;
const int txPin = 11;
const int ddPin = 12;

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
#define d_ideal1         70 //Distance to iRobot needs to keep from agent 1
#define d_ideal2         100 //Distance to iRobot needs to keep from agent 2 
#define k_v             15
#define time_step       7
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
#define k_d             0.2
const int track_tol = 2;
/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

/* estimation related parameters */
float D1[2], D2[2]; //X, Y position of the agents
float distance_abs[2]  = {0, 0}; //distance to the agents
float angle[2]; //angle of the agents
float relative_error[2];

/* Control safe switch */
int safe_switch = 0;


struct formInfo {
  int velocity;
  double angle;
};


struct driveInfo {
  int vel;
  int timestep;
};

void setup() {
  // Initialize serial
  Serial.begin(9600);    // serial terminal on the computer
  Serial1.begin(115200); // sweep device
  device.setMotorSpeed(MOTOR_SPEED_CODE_1_HZ); //This configuration gives the most points per angle
  device.setSampleRate(SAMPLE_RATE_CODE_1000_HZ); //Lowering speed, and increasing the rate caused troubles


  // initialize iRobot             */
  Serial2.begin(19200); //iRobot
  delay(2000); // NEEDED!!!! To let the robot initialize
  Serial2.write(128); // This command starts the OI. You must always send the Start command before sending any other commands to the OI
  delay(50);
  Serial2.write(131); // Go into save mode, i.e. whenever robot is picked up it stops (see p. 6 of Create's Open Interface documentation)
  delay(50);

  // initialize counter variables and reset the current state
  reset_Sweep();
}

// Resets the variables and state so the sequence can be repeated
void reset_Sweep()
{
  scanCount = 0;
  sampleCount = 0;
  // reset the sensor
  device.reset();
  delay(50);
  Serial.flush();
  Serial.println("\n\nWhenever you are ready, type \"start\" to to begin the sequence...");
  delay(1000);
  Serial.write("start");
}
// initialize all timers         */
int t = millis();

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//=========@@@=@@@=====@=====@@@===@@===@===========@========@@@@=====@@@@=====@@@@=========
//=========@==@==@====@=@=====@====@=@==@===========@=======@====@===@====@====@===@========
//=========@==@==@===@@@@@====@====@==@=@===========@=======@====@===@====@====@@@@=========
//=========@=====@===@===@===@@@===@===@@===========@@@@@====@@@@=====@@@@=====@============
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
void loop()
{
  Serial.println("1" );
  sampleCount = 0;
  device.startScanning();
  Serial.println("2" );
  while (sampleCount < 1000) {
    bool success = false;
    ScanPacket reading = device.getReading(success);
    if (success)
    {
      // store the info for this sample
    syncValues[sampleCount] = reading.isSync();
    angles[sampleCount] = reading.getAngleDegrees();
    distances[sampleCount] = reading.getDistanceCentimeters();
    signalStrengths[sampleCount] = reading.getSignalStrength();

      // increment sample count
      sampleCount++;
    }
  }
  Serial.println("3" );
  device.stopScanning();
  Serial.println("4" );

  int indexOfFirstSyncReading = 0;
  int new_scan_ctr = 0;
  int matrix_size = 0; //angle, distance array size
  // don't print the trailing readings from the first partial scan
  while (!syncValues[indexOfFirstSyncReading])
  {
    indexOfFirstSyncReading++;
  }
  // print the readings for all the complete scans
  Serial.print("Index of first reading: ");
  Serial.println(indexOfFirstSyncReading);
  for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
  {
    if (syncValues[i])
    {
      new_scan_ctr++;
      if (new_scan_ctr == 2) {
        matrix_size = i - indexOfFirstSyncReading;
        Serial.println(matrix_size);
      }
      Serial.println("\n----------------------NEW SCAN----------------------");
    }
    Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
  }
  float angles_new[500] = {0};            // in degrees (accurate to the millidegree)
  int distances_new[500] = {0};      // in cm
  for (int i = indexOfFirstSyncReading; i < (matrix_size + indexOfFirstSyncReading); i++)
  {
    angles_new[i - indexOfFirstSyncReading] = angles[i];
    distances_new[i - indexOfFirstSyncReading] = distances[i];
  }
  DBSCAN_Main(angles_new, distances_new, matrix_size); //Sends DBSCAN angles from 0 to 360 degrees, corresponding distances, and size of those arrays

  t = millis();

  formAtion();
  reset_Sweep();
}

/////////////////////////////////////////////////////////
////////////////  DBSCAN MAIN ////////////////////////////////
/////////////////////////////////////////////////////////

void DBSCAN_Main(float Angle[], int Distance[], int n)
{
  Serial.println("Inside the DBSCAN_Main");
  float X[600][2];
  int IDX_Max = 0; //Maximum value in IDX
  for (int i = 0; i < n; i++) {
    //change to Cartesian
    X[i][0] = cos(Angle[i] * PI / 180) * Distance[i]; 
    X[i][1] = sin(Angle[i] * PI / 180) * Distance[i];
  }

  /************** Filters *********************/
  Serial.println("After first loop in DBSCAN_Main");
  // Max distance = 300 cm
  for (int i = 0; i < n; i++)
  {
    //anything further than 250 cm and below 5 cm are noise (2.5 m is all we need to test the iRobots)
    if  ((pow((pow((X[i][0]), 2) + pow((X[i][1]), 2)), 0.5) > 200) || (pow((pow((X[i][0]), 2) + pow((X[i][1]), 2)), 0.5) < 5))
    {
      X[i][0] = 0;
      X[i][1] = 0;
    }
  }
  Serial.println("About to enter DBSCAN");

  // Run DBSCAN Clustering Algorithm

  int IDX[600] = {0}; //array that assigns cluster number to each X, Y point
  
  IDX_Max = DBSCAN(X, n, IDX); //DBSCAN function
  Serial.print("IDX MAX: ");
  Serial.println(IDX_Max);
  // Taking out clusters with more than 9 points
  float avg_dist[100] = {0};
  float X_avg[100] = {0};
  float Y_avg[100] = {0};
  float X_avg_temp = 0;
  float Y_avg_temp = 0;
  float avg_dist_temp = 0;
  float avg_angle[100] = {0};
  float avg_dist2[100] = {0};
  float avg_angle_temp = 0;
  float avg_dist2_temp = 0; //from distance matrix
  
  for (int i = IDX_Max - 1; i >= 0; i--)
  {
    Serial.println(i);
    int k = 0;
    for (int j = 0; j < n; j++)
    {
      if (IDX[j] == i)
      {
        k++;
        X_avg_temp = X[j][0] + X_avg[i];
        Y_avg_temp = X[j][1] + Y_avg[i];
        X_avg[i] = X_avg_temp;
        Y_avg[i] = Y_avg_temp;
        avg_dist_temp = pow((pow((X[j][0]), 2) + pow((X[j][1]), 2)), 0.5) + avg_dist[i];
        avg_dist[i] = avg_dist_temp;
        avg_dist2_temp = avg_dist2[i] + Distance[j];
        avg_dist2[i] = avg_dist2_temp;
        avg_angle_temp = avg_angle[i] + Angle[j];
        avg_angle[i] = avg_angle_temp;
      }
    }
    Serial.println("5");
    X_avg[i] = X_avg[i]/(float)k;
    Y_avg[i] = Y_avg[i]/(float)k;
    avg_dist[i] = avg_dist[i] / (float)k;
    avg_dist2[i] = avg_dist2[i] / (float)k;
    avg_angle[i] = avg_angle[i] / (float)k;
    Serial.println("6");
    /* k - number of points in cluster, 9 is max at closest distance,
      800 is 200 cm times 4 cluster points at 200 distance (measure
      again) */
    if ((k > 9) || (k * avg_dist[i] > 800))
    {
      X_avg[i] = 0;
      Y_avg[i] = 0;
      avg_dist[i] = 0;
      for (int j = 0; j < n; j++)
      {
        if (IDX[j] == i)
        {
          IDX[j] = 0;
        }
      }
    }
    Serial.println("7");
  }
  int final_ctr = 0;
  Serial.println("8");
  // Identify agents as first two clusters and send their position, and distance further to formation()
  while(avg_dist[final_ctr] == 0){
    final_ctr++;
  }
  Serial.println("9");
  Serial.println("final_ctr");
  distance_abs[0] = avg_dist[final_ctr];
  D1[0] = X_avg[final_ctr];
  D2[0] = Y_avg[final_ctr];
  avg_dist2[0] = avg_dist2[final_ctr];
  avg_angle[0] = avg_angle[final_ctr];
  final_ctr++;
  Serial.println("10");
  while(avg_dist[final_ctr] == 0){
    final_ctr++;
  }
  Serial.println("11");
  distance_abs[1] = avg_dist[final_ctr];
  D1[1] = X_avg[final_ctr];
  D2[1] = Y_avg[final_ctr];
  avg_dist2[1] = avg_dist2[final_ctr];
  avg_angle[1] = avg_angle[final_ctr];
  Serial.print("Matrix size: ");
  Serial.println(n);
  Serial.println("distance_abs[0]: " + String(distance_abs[0]) + ", distance_abs[1]: " + String(distance_abs[1]) + ", D1[0]" + String(D1[0]) + ", D2[0]" + String(D2[0]) + ", D1[1]" + String(D1[1]) + ", D2[1]" + String(D2[1]));
  Serial.println("Distance2[0]" + String(avg_dist2[0]) + ", Angle[0]" + String(avg_angle[0])  + ", Distance2[1]" + String(avg_dist2[1]) + ", Angle[1]" + String(avg_angle[1]));
  
}

/////////////////////////////////////////////////////////
////////////////  DBSCAN ////////////////////////////////
/////////////////////////////////////////////////////////

int DBSCAN(float X[][2], int n, int IDX[])///////////////
{
  int C = 0;
  
  int IDX_Max = 0; //Maximum value in IDX
  float D[600][11]; //Matrix that contains distance from current point to 5 points to the left and 5 points to the right

  for (int i = 0; i < 600; i++){
    for (int j = 0; j < 11; j++){
      D[i][j] = 0;
    }
  }
  int t_ctr = 0;
  //D = pdist2(X, X);
  
  for (int i = 0; i < n; i++){
    for (int j = 0; j < 11; j++){
      t_ctr = i + j - 5;
      if ( t_ctr < 0){
        t_ctr = n + t_ctr;
      }
      else if (t_ctr >= n){
        t_ctr = n - t_ctr; 
      }
      D[i][j] = pow((pow((X[t_ctr][0] - X[i][0]), 2) + pow((X[t_ctr][1] - X[i][1]), 2)), 0.5);
    }
  }
  // D matrix logic is totally correct, unless you want to increase j from 11 to higher range
  
  int visited[600] = {0}; //Since I made arrays with constant size instead of making them dynamic, I chose 600 to be safe number
  int isnoise[600] = {0}; //because usually array size of angles sent is around 400
  
  int Neighbors[1000] = {0};//tekseru kerek, 400den aspauy kerek
  
  int Neighbors_Temp[11] = {0};
  int Neighbors_ctr = 0;

  for (int i = 0; i < n; i++)
  {
    if (visited[i] == 0)
    {
      visited[i] = 1;      
      Neighbors_ctr = RegionQuery(Neighbors_Temp, D, i); //check if array changes by reference
      for (int ctr= 0; ctr < Neighbors_ctr; ctr++)
      {
        Neighbors[ctr] = Neighbors_Temp[ctr];
      }
      
      if (Neighbors_ctr < MinPts)
      {
        // X(i,:) is NOISE
        isnoise[i] = 1;
      }
      else
      {
        C=C+1;
        IDX_Max = ExpandCluster(i, Neighbors, Neighbors_ctr, C, n, D, IDX, visited); //can be changed to array that contains Neighbors
      }

    }

    

  }
  return IDX_Max;
}
/////////////////////////////////////////////////////////
////////////////  EXPAND CLUSTER /////////////////////////
/////////////////////////////////////////////////////////

int ExpandCluster(int i, int Neighbors[], int s, int C, int n, float D[][11], int IDX[], int visited[])////////////////
{
  IDX[i] = C;
  int IDX_Max = 1;
  if (IDX_Max < C){
    IDX_Max = C;
  }
  int Neighbors2_ctr = 0;
  int Temp_N[1000] = {0}; //[s]
  
  for (int ctr = 0; ctr < s; ctr++){
    Temp_N[ctr] = Neighbors[ctr];
  }
  int Neighbors2[11] = {0};
  int k = 0;
  int j = 0;
  int true_val = 1;
while (true_val){
  for (int ctr = 0; ctr < s; ctr++){
    Neighbors[ctr] = Temp_N[ctr];
  }
  
  if (((Temp_N[k] + i - 5) >= 0) && ((Temp_N[k] + i - 5) < n)) {
      j = Temp_N[k] + i - 5;
    }
    else if (((Temp_N[k] + i - 5) >= n)) {
      j = Temp_N[k] + i - 5 - n;
    }
    else {
      j = Temp_N[k] + i - 5 + n;
    }

    if (visited[j] == 0)
    {
      visited[j] = 1;
      Neighbors2_ctr = RegionQuery(Neighbors2, D, j);
      for (int ctr_n = 0; ctr_n < Neighbors2_ctr; ctr_n++){
        if ((Neighbors2[ctr_n] + (j - i)) < 0){
          Neighbors2[ctr_n] = Neighbors2[ctr_n] + (j - i) + n;
        }
        else if ((Neighbors2[ctr_n] + (j - i)) >= n){
          Neighbors2[ctr_n] = Neighbors2[ctr_n] + (j - i) - n;
        }
        else{
          Neighbors2[ctr_n] = Neighbors2[ctr_n] + (j - i);
        }
      }
      
      if (Neighbors2_ctr >= MinPts)
      {
        for (int ctr = 0; ctr < s; ctr++){
          Temp_N[ctr] = Neighbors[ctr];
        }
        for (int ctr = 0; ctr < Neighbors2_ctr; ctr++){
          Temp_N[ctr+s] = Neighbors2[ctr];
        }
        
        s = Neighbors2_ctr + s;
        ////Neighbors = [Neighbors Neighbors2]; //#ok//////////////
      }

    }
    
    if (IDX[j] == 0)
    {
      IDX[j] = C;
    }
    k++;
    if (k >= s)
    {
      true_val = 0;
    }
  }
  return IDX_Max;
}

/////////////////////////////////////////////////////////
////////////////  Region Query /////////////////////////
/////////////////////////////////////////////////////////

int RegionQuery(int Neighbors[], float D[][11], int i)
{
  int k = 0;
  for (int j = 0; j < 11; j++){
    if ((D[i][j] <= epsilon)){
      Neighbors[k] = j;//j-di tekser, j+1 bolyp jiberip jatqan joq pa
      k++;
    }
     
  }
  return k;
}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//==============@@@@====@===@====@==================@@@@@=====@@@=====@@===@====@@@@========
//==============@@==@===@===@====@@@================@========@===@====@=@==@====@@==@=======
//=============@==@@====@===@====@==@===============@@@@@====@========@==@=@===@==@@========
//==============@@@@=====@@@=====@@@================@=========@@@=====@===@@====@@@@========
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||



struct formInfo formCalcVel() {
  int     V1, V2;
  int         radius;
  int         t_r; // local variable to find how long need for rotation
  double      k1, k2, d1, d2, form_velocity;
  double      form_angle;
  struct formInfo formInfoLoc;

  d1 = distance_abs[0]; ///////////////////////////////////
  d2 = distance_abs[1]; ///////////////////////////////////
  k1 = sq(d1) - sq(d_ideal1);
  k2 = sq(d2) - sq(d_ideal2);
  V1 = round(D1[0]  * k1 + D1[1] * k2); // velocity component x
  V2 = round(D2[0]  * k1 + D1[1] * k2); /////////////////////////////

  form_velocity = float(k_v) * float(sqrt(sq(V1 / 1000000.0) + sq(V2 / 1000000.0)));
  Serial.print("Check vel: ");
  Serial.println(form_velocity);
  form_angle = atan2(V2, V1);

  if (form_angle > PI / 2) {
    form_angle = form_angle - PI;
    form_velocity = - form_velocity;
  }
  else if (form_angle < -PI / 2) {
    form_angle = form_angle + PI;
    form_velocity = - form_velocity;
  }

  formInfoLoc.angle = form_angle;
  formInfoLoc.velocity = form_velocity;

  return formInfoLoc;
}

int formRotTime(float angle) {
  int t_r;
  double angle_d;
  angle_d = - angle *  180 / PI;
  if (angle_d > 4.5) t_r = round(17.7930 * angle_d + 79.2839);
  if (angle_d < -4.5) t_r = -round(17.7930 * -angle_d + 79.2839);
  if (angle_d < 4.5 && angle_d > -4.5) t_r = 0;
  return t_r;
}

void formAtion() {
  struct formInfo formInfoIRobot;
  int vel, t_r;
  struct driveInfo drieInfo1;
  formInfoIRobot = formCalcVel();

  drieInfo1 = driveDistToVel(formInfoIRobot.velocity);
  vel = drieInfo1.vel;



  int t_form = millis();

  t_r = formRotTime(formInfoIRobot.angle);
  Serial.print(" angle: ");
  Serial.print(formInfoIRobot.angle);
  Serial.print(" time rotation: ");
  Serial.print(t_r);
  Serial.print(" velocity: ");
  Serial.print(vel);
  Serial.print(" d distance: ");
  Serial.println(formInfoIRobot.velocity);

  t_r = constrain(-500, 500, t_r);
  if (t_r > 0) {
    while ( (millis() - t_form) < t_r)
    {
      if (safe_switch) drive(100, 1);
      // rotate left
    }
  }
  else {
    while ( (millis() - t_form) < (-t_r) )
    {
      if (safe_switch) drive(100, -1);
      //rotate right 
    }
  }

  driveRest();

  if ((millis() - t_form) < 1000) {
    delay(1000 - (millis() - t_form) );
  }
  delay(500); // the delay allows the Analog to Digital converter IC to recover for the next reading.

  t_form = millis();
  while ( (millis() - t_form) < drieInfo1.timestep)
  {
    if (safe_switch) driveStraight(vel);
  }

  driveRest(); // the delay allows the Analog to Digital converter IC to recover for the next reading.

  if (time_step > (millis() - t)) // delay up to  7 second
  {
    delay(time_step - (millis() - t));
  }

  safe_switch = 1;
}

void driveStraight(int velocity) {
  byte cmd[] = {(byte) 137,
                (byte)highByte(velocity),
                (byte)lowByte(velocity),
                (byte)128,
                (byte)0
               }; ;    // these are the bytes for the special case of driving straight

  for (int i = 0; i < 5; i++) {
    Serial2.write(cmd[i]);
  }

}

void drive(int velocity, int radius) {
  byte cmd[] = {(byte) 137,
                (byte)highByte(velocity),
                (byte)lowByte(velocity),
                (byte)highByte(radius),
                (byte)lowByte(radius)
               };

  for (int i = 0; i < 5; i++) {
    Serial2.write(cmd[i]);
  }
}

void driveRest() {
  delay(100); // the delay allows the Analog to Digital converter IC to recover for the next reading.
  drive(0, 0);
}



struct driveInfo driveDistToVel(int  dx) {
  int velocity;
  struct driveInfo drieInfo1;


  dx = constrain(dx, -41, 41);
  //  Serial.print(" dx ");
  //  Serial.print(dx);
  if (dx > 0)
  {
    //    Serial.println(" dx > 0 ");
    velocity = dx / 6 + 1;
  }
  if (dx == 0)
  {
    //    Serial.println(" dx = 0 ");
    velocity = 0;
  }
  if (dx < 0)
  {
    //    Serial.println(" dx < 0 ");
    velocity =  -(abs(dx) / 6 + 1);
  }
  //  Serial.print(" dvelocity ");
  //  Serial.println(dx);
  switch (velocity) {
    case -1:
      drieInfo1.vel = -20;  //do something when velocity equals -1
      break;
    case -2:
      drieInfo1.vel = -40;  //do something when velocity equals -2
      break;
    case -3:
      drieInfo1.vel = -80;  //do something when velocity equals -3
      break;
    case -4:
      drieInfo1.vel = -120; //do something when velocity equals -4
      break;
    case -5:
      drieInfo1.vel = -140; //do something when velocity equals -5
      break;
    case -6:
      drieInfo1.vel = -160; //do something when velocity equals -6
      break;
    case -7:
      drieInfo1.vel = -200; //do something when velocity equals -7
      break;
    case 0:
      drieInfo1.vel = 0;   //do something when velocity equals 0
      break;
    case 1:
      drieInfo1.vel = 20;  //do something when velocity equals 1
      break;
    case 2:
      drieInfo1.vel = 40;  //do something when velocity equals 2
      break;
    case 3:
      drieInfo1.vel = 80;  //do something when velocity equals 3
      break;
    case 4:
      drieInfo1.vel = 120; //do something when velocity equals 4
      break;
    case 5:
      drieInfo1.vel = 140; //do something when velocity equals 5
      break;
    case 6:
      drieInfo1.vel = 160; //do something when velocity equals 6
      break;
    case 7:
      drieInfo1.vel = 200; //do something when velocity equals 7
      break;
    default:
      drieInfo1.vel = 0;
      break;
  }
  drieInfo1.timestep = dx * 2000 / (velocity * 6);
  return (drieInfo1);
}


