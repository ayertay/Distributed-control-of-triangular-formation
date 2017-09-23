
#include <Sweep.h>
int epsilon = 10;
int MinPts = 1;

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
   // Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
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
  
  IDX_Max = DBSCAN(X, n, IDX); //DBSCAN function
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
  for (int i = 0; i < n; i++){
    Serial.println("X[1]: " + String(X[i][0]) + ", X[2]: " + String(X[i][1]) + ", IDX: " + String(IDX[i]));
  }
  
}

/////////////////////////////////////////////////////////
////////////////  DBSCAN ////////////////////////////////
/////////////////////////////////////////////////////////

int DBSCAN(float X[][2], int n, int IDX[])///////////////
{
  int C = 0;
  
  int IDX_Max = 0; //Maximum value in IDX
  float D[600][11];

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
  
  //int visited[n] = {0};//// also dynamic
  int visited[600] = {0};
  int isnoise[600] = {0};
  
  int Neighbors[1000] = {0};//tekseru kerek, 400den aspauy kerek
  
  int Neighbors_Temp[11] = {0};
  int Neighbors_ctr = 0;

  for (int i = 0; i < n; i++)
  {
    if (visited[i] == 0)
    {
      visited[i] = 1;
      
      Neighbors_ctr = RegionQuery(Neighbors_Temp, D, i);//check if array changes by reference
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
  int Temp_N[1000] = {0}; //[s]
  
  for (int ctr = 0; ctr < s; ctr++){
    Temp_N[ctr] = Neighbors[ctr];
  }
  int Neighbors2[11] = {0};
  int k,j = 0; //it was
  while (true)
{
/////////////// Try to find a way to change to fixed array instead of dynamic////////////
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
    Serial.println(j);
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
      Neighbors[k] = j;//j-di tekser, j+1 bolyp jiberip jatqan joq pa
      k++;
    }
     
  }
  return k;
}

