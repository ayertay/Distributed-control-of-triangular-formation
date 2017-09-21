
/*
  Scanse Sweep Arduino Library Examples

  MegaSerialPrinter:
      - Example sketch for using the Scanse Sweep with the Arduino Mega 2560.
        Collects 3 complete scans, and then prints the sensor readings
      - Assumes Sweep sensor is physically connected to Serial #1 (RX1 & TX1)
        - For the sweep's power, ground, RX & TX pins, follow the connector
          pinouts in the sweep user manual located here:
          http://scanse.io/downloads
        - Be sure to connect RX_device -> TX_arduino & TX_device -> RX_arduino
      - For best results, you should provide dedicated external 5V power to the Sweep
        rather than using power from the Arduino. Just be sure to connect the ground
        from the power source and the arduino. If you are just experimenting, you can
        run the sweep off the 5V power from the Arduino with the Arduino receiving power
        over USB. However this has only been tested with an external powered USB hub.
        It is possible that using a low power USB port (ex: laptop) to power the
        arduino & sweep together will result in unexpected behavior.
      - Note that running off of USB power is not entirely adequate for the sweep,
        so the quantity and qaulity of sensor readings will drop. This is OK for
        this example, as it is only meant to provide some visual feedback over
        the serial monitor.
      - In your own projects, be sure to use dedicated power instead of the USB.

  Created by Scanse LLC, February 21, 2017.
  Released into the public domain.
*/

#include <Sweep.h>
int epsilon = 10;
int MinPts = 2;

// Create a Sweep device using Serial #1 (RX1 & TX1)
Sweep device(Serial1);
// Scan packet struct, used to store info for a single reading
ScanPacket reading;

// keeps track of how many scans have been collected
int scanCount = 0;
// keeps track of how many samples have been collected
int sampleCount = 0;

// Arrays to store attributes of collected scans
bool syncValues[1000];         // 1 -> first reading of new scan, 0 otherwise
float angles[1000];            // in degrees (accurate to the millidegree)
int distances[1000];      // in cm
int signalStrengths[1000]; // 0:255, higher is better



void setup()
{
  // Initialize serial
  Serial.begin(9600);    // serial terminal on the computer
  Serial1.begin(115200); // sweep device

  device.setMotorSpeed(MOTOR_SPEED_CODE_1_HZ);
  device.setSampleRate(SAMPLE_RATE_CODE_1000_HZ);

  // initialize counter variables and reset the current state
  reset_Sweep();
}

// Loop functions as an FSM (finite state machine)
void loop()
{
  sampleCount = 0;
  device.startScanning();
  while (sampleCount < 1000) {
    if (device.getReading(reading))
    {
      // store the info for this sample
      syncValues[sampleCount] = reading.bIsSync;
      angles[sampleCount] = reading.angle;
      //Serial.println(String(angles[sampleCount], 3) + ", Sample Count: " + String(sampleCount));
      distances[sampleCount] = reading.distance;
      signalStrengths[sampleCount] = reading.signalStrength;

      // increment sample count
      sampleCount++;
    }
  }
  device.stopScanning();


  int indexOfFirstSyncReading = 0;
  int new_scan_ctr = 0;
  int matrix_size = 0;
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
      if(new_scan_ctr == 2){
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
  DBSCAN_Main(angles_new, distances_new, matrix_size);
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

/////////////////////////////////////////////////////////
////////////////  DBSCAN MAIN ////////////////////////////////
/////////////////////////////////////////////////////////

void DBSCAN_Main(float Angle[], int Distance[], int n)
{
  
  float X[600][2];
  int IDX_Max = 0; //Maximum value in IDX
  for (int i = 0; i < n; i++) {
    X[i][0] = cos(Angle[i] * PI / 180) * Distance[i];
    X[i][1] = sin(Angle[i] * PI / 180) * Distance[i];
  }

  /************** Filters *********************/

  // Max distance = 300 cm
  for (int i = 0; i < n; i++)
  {
    if  ((pow((pow((X[i][0]), 2) + pow((X[i][1]), 2)), 0.5) > 300) || (pow((pow((X[i][0]), 2) + pow((X[i][1]), 2)), 0.5) < 5))
    {
      X[i][0] = 0;
      X[i][1] = 0;
    }
  }

  // Run DBSCAN Clustering Algorithm

//  int epsilon = 10;
//  int MinPts = 2;
  int IDX[600] = {0};
  
  IDX_Max = DBSCAN(X, n, epsilon, MinPts, IDX); //DBSCAN function
  Serial.println(IDX_Max);
  // Taking out clusters with more than 9 points
  /*float avg_dist[100] = {0};
  
  for (int i = 30 - 1; i >= 0; i--)
  {
    int k = 0;
    for (int j = 0; j < n; i++)
    {
      if (IDX[j] == i)
      {
        k++;
        avg_dist[i] = pow((pow((X[j][1]), 2) + pow((X[j][2]), 2)), 0.5 + avg_dist[i]);
      }
    }
    avg_dist[i] = avg_dist[i] / k;
    /* k - number of points in cluster, 9 is max at closest distance,
      800 is 200 cm times 4 cluster points at 200 distance (measure
      again) */
    /*if ((k > 9) || (k * avg_dist[i] > 800))
    {
      for (int j = 0; j < n; i++)
      {
        if (IDX[j] == i)
        {
          IDX[j] = 0;
        }
      }
    }
  }*/
  Serial.print("Matrix size: ");
  Serial.println(n);
//  for (int i = 0; i < n; i++){
//    Serial.println("X[1]: " + String(X[i][0]) + ", X[2]: " + String(X[i][1]) + ", IDX: " + String(IDX[i]));
//  }
  
}

/////////////////////////////////////////////////////////
////////////////  DBSCAN ////////////////////////////////
/////////////////////////////////////////////////////////

int DBSCAN(float X[][2], int n, int epsilon, int MinPts, int IDX[])///////////////
{
  int C = 0;
  
  int IDX_Max = 0; //Maximum value in IDX
  float D[600][11];

  for (int i = 0; i < 600; i++){
    for (int j = 0; j < 11; j++){
      D[i][j] = 0;
    }
  }
  
  //D = pdist2(X, X);
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 6; j++) {
      D[i][j] = pow((pow((X[n + j - 5][0] - X[i][0]), 2) + pow((X[n + j - 5][1] - X[i][1]), 2)), 0.5);
    }
    for (int j = 6; j < 11; j++) {
      D[i][j] = pow((pow((X[j - 5][0] - X[i][0]), 2) + pow((X[j - 5][1] - X[i][1]), 2)), 0.5); //consider changing 5 to 6
    }
  }

  for (int i = 5; i < (n - 5); i++) {
    for (int j = 0; j < 11; j++) {
      D[i][j] = pow((pow((X[i + j - 5][0] - X[i][0]), 2) + pow((X[i + j - 5][1] - X[i][1]), 2)), 0.5);
    }
  }
  for (int i = (n - 5); i < n; i++) {
    for (int j = 0; j < 6; j++) {
      D[i][j] = pow((pow((X[i + j - 5][0] - X[i][0]), 2) + pow((X[i + j - 5][1] - X[i][1]), 2)), 0.5);
    }
    for (int j = 6; j < 11; j++) {
      D[i][j] = pow((pow((X[j - 5][0] - X[i][0]), 2) + pow((X[j - 5][1] - X[i][1]), 2)), 0.5); //consider changing 5 to 6
    }
  }
  for (int i = 0; i < n; i++){
    Serial.println("D[1]: " + String(D[i][0]) + ", D[2]: " + String(D[i][1]) + ", D[3]: " + String(D[i][2]) + ", D[4]: " + String(D[i][3]) + ", D[5]: " + String(D[i][4]) + ", D[6]: " + String(D[i][5]) + ", D[7]: " + String(D[i][6]) + ", D[8]: " + String(D[i][7]) + ", D[9]: " + String(D[i][8]) + ", D[10]: " + String(D[i][9]) + ", D[11]: " + String(D[i][10]));
  }
  //int visited[n] = {0};//// also dynamic
  int visited[600] = {0};
  int isnoise[600] = {0};
  
  int Neighbors[100] = {0};
  
  int Neighbors_Temp[11] = {0};
  int Neighbors_ctr = 0;

  for (int i = 0; i < n; i++)
  {
    if (!visited[i])
    {
      visited[i] = 1;
      
      Neighbors_ctr = RegionQuery(Neighbors_Temp, D, i);
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
        C++;
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

  //////////// Experimental algorithm /////////////
  int New_Neighbor[100] = {0}; //[s]

  
  for (int ctr = 0; ctr < s; ctr++){
    New_Neighbor[ctr] = Neighbors[ctr];
  }
  
  int Temp_N[100] = {0}; //[s]
  
  for (int ctr = 0; ctr < s; ctr++){
    Temp_N[ctr] = Neighbors[ctr];
  }
  int Neighbors2[100] = {0};
  int Neighbors2_Temp[11] = {0};
  ///////////////////////////////////////////////
  int k = 1;
  int j = 0;
  while (true)
{
/////////////// Try to find a way to change to fixed array instead of dynamic////////////
  for (int ctr = 0; ctr < s; ctr++){
    New_Neighbor[ctr] = Temp_N[ctr];
  }

  
  if (((Temp_N[k] + i - 6) >= 0) && ((Temp_N[k] + i - 6) < n)) {
      j = Temp_N[k] + i - 6;
    }
    else if (((Temp_N[k] + i - 6) >= n)) {
      j = Temp_N[k] + i - 6 - n;
    }
    else {
      j = Temp_N[k] + i - 6 + n;
    }

    if (!visited[j])
    {
      visited[j] = 1;
      Neighbors2_ctr = RegionQuery(Neighbors2_Temp, D, i);
      for (int ctr_n = 0; ctr_n < Neighbors2_ctr; ctr_n++){
        Neighbors2[ctr_n] = Neighbors2_Temp[ctr_n];
      }
    
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
          Temp_N[ctr] = New_Neighbor[ctr];
        }
        for (int ctr = s; ctr < (Neighbors2_ctr + s); ctr++){
          Temp_N[ctr] = Neighbors2[ctr];
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
    if (k > s)
    {
      break;
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
    if ((D[i][j] <= epsilon) && (D[i][j] > 0)){
      Neighbors[k] = D[i][j];
      k++;
    }
     
  }
  return k;
}

