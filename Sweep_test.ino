
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

// Create a Sweep device using Serial #1 (RX1 & TX1)
Sweep device(Serial1);
// Scan packet struct, used to store info for a single reading
ScanPacket reading;

// keeps track of how many scans have been collected
uint8_t scanCount = 0;
// keeps track of how many samples have been collected
uint16_t sampleCount = 0;

// Arrays to store attributes of collected scans
bool syncValues[1000];         // 1 -> first reading of new scan, 0 otherwise
float angles[1000];            // in degrees (accurate to the millidegree)
uint16_t distances[1000];      // in cm
uint8_t signalStrengths[1000]; // 0:255, higher is better



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
  // don't print the trailing readings from the first partial scan
  while (!syncValues[indexOfFirstSyncReading])
  {
    indexOfFirstSyncReading++;
  }
  // print the readings for all the complete scans
  for (int i = indexOfFirstSyncReading; i < sampleCount; i++)
  {
    if (syncValues[i])
    {
      Serial.println("\n----------------------NEW SCAN----------------------");
    }
    Serial.println("Angle: " + String(angles[i], 3) + ", Distance: " + String(distances[i]) + ", Signal Strength: " + String(signalStrengths[i]));
  }
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
}

void DBSCAN_Main()
{
  int Angle[] = Matrix with angle data;
  int Distance[] = Matrix with distance data;

  float X[][2];
  for (int i = 0; i < Y; i++) {
    X[i][1] = cos(Angle[i] * PI / 180) * Distance[i];
    X[i][2] = sin(Angle[i] * PI / 180) * Distance[i];
  }

  /************** Filters *********************/

  // Max distance = 300 cm
  for (int i = 0; i < Y; i++)
  {
    if  (pow((pow((X[i][1]), 2) + pow((X[i][2]), 2)), 0.5) > 300)
    {
      X[i][1] = 0;
      X[i][2] = 0;
    }
  }

  // Run DBSCAN Clustering Algorithm

  int epsilon = 10;
  int MinPts = 2;
  IDX = DBSCAN(X, epsilon, MinPts); //DBSCAN function

  // Taking out clusters with more than 9 points
  float avg_dist = zeros(max(IDX), 1);
  for (i = max(IDX) - 1; i >= 0; i--)
  {
    int k = 0;
    for (int j = 0; j < length(IDX); i++)
    {
      if (IDX[j] == i)
      {
        k++;
        avg_dist[i] = pow((pow((X[j][1]), 2) + pow((X[j][2]), 2)), 0.5 + avg_dist[i];
      }
    }
    avg_dist[i] = avg_dist[i] / k;
    /* k - number of points in cluster, 9 is max at closest distance,
      800 is 200 cm times 4 cluster points at 200 distance (measure
      again) */
    if ((k > 9) || (k * avg_dist[i] > 800))
    {
      for (int j = 0; j < length(IDX); i++)
      {
        if (IDX[j] == i)
        {
          IDX[j] = 0;
        }
      }
    }
  }

}

int DBSCAN()
{
  int C = 0;

  n = size(X, 1);

  IDX = zeros(n, 1);

  D = pdist2(X, X);

  visited = false(n, 1);
  isnoise = false(n, 1);

  for (int i = 0; i < n; i++)
  {
    if ~visited(i)
    {
      visited(i) = true;

      Neighbors = RegionQuery(i);
      if numel(Neighbors) < MinPts
      {
        // X(i,:) is NOISE
        isnoise(i) = true;
      }
      else
      {
        C++;
        ExpandCluster(i, Neighbors, C);
      }

    }

  }

}

void ExpandCluster(i, Neighbors, C)
{
  IDX(i) = C;

  k = 1;
  while true
  {
    j = Neighbors(k);

    if ~visited(j)
    {
      visited(j) = true;
      Neighbors2 = RegionQuery(j);
      if numel(Neighbors2) >= MinPts
      {
        Neighbors = [Neighbors Neighbors2]; //#ok
      }
    }
    if IDX(j) == 0
    {
      IDX(j) = C;
    }

    k++;
    if k > numel(Neighbors)
    {
      break;
    }
  }
}

void Neighbors = RegionQuery(i)
{
  Neighbors = find(D(i, : ) <= epsilon);
}


