#include <Math.h>
#include <stdlib.h>
#include <Servo.h>
int drivedist = 0;
int MaxLines = 320;

double CollisionBufferSize = 15; // in cm
double posx = 0.0;
double posy = 0.0;
double heading = 0.0;
int list_count = 0;
double nineDelay = 0.0;
int sample_size = 300;
double ** coords{sample_size};
int count = 0;
const int READ_LENGTH = 9;
double sLP[4][2] = {{365, 5}, {39.5, 121.5}, {159.5, 201.5}, {231.5, 311.5}};
byte outputOn[5] = {0x5A, 0x05, 0x07, 0x01, 0x00}; // enabling output from TF_Luna. Change the fourth value to 0x00 to diable output
byte format[5] = {0x5A, 0x05, 0x05, 0x06, 0x00}; // setting output to 9 byte/mm format
byte frequencySet[6] = {0x5A, 0x06, 0x03, 0x02, 0x00, 0x00}; // setting putput frequency. Fourth (and fith) value is frequency in Hz.
byte Recieved[9];
byte setTrigger[6] = {0x5A, 0x06, 0x03, 0x00, 0x00, 0x00}; // puts TF_Luna into trigger mode
byte trigger[4] = {0x5A, 0x04, 0x04, 0x00};
double ang = 0.0;
double angle = 0.0;

Servo myservo;

double d_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 




class GuideLines {
  public:
    bool Taken = false;  
    double StartPt[2];
    double EndPt[2];
    bool VertSlope = false;
    bool HorizSlope = false;

};

GuideLines* MemLines = new GuideLines[MaxLines];



int getNavLinesNum(){
  for(int i = 0; i < MaxLines; i++){
      if(MemLines[i].Taken == true){
          return(i+1);
      }
  }
  return(MaxLines);
}

// :(

double get_dist_from_pairs(double l1[2], double l2[2]){
    return(sqrt(pow(abs(l1[0] - l2[0]), 2) + pow(abs(l1[1] - l2[1]), 2)));
  }

  double * trig_calc (double dist, double angl){
    double rads = angl/57.29577953;
    double x = cos(rads)*dist;
    double y = sin(rads)*dist;
    if (angl <= 90){
      
      }
    double xylist[2];
    xylist[0] = x;
    xylist[1] = y;
    return(xylist); 
    }


double get_real_angle(double angle){

  if(angle + heading > 360){
    angle = (360-heading);
  }
  else{angle = angle+heading;}
  return(angle);
}

void AddGuideLine(double StartPoint[2], double EndPoint[2], bool Vert, bool Horiz){
  int z = getNavLinesNum();
  double sentSlope = (EndPoint[1]-StartPoint[1])/(EndPoint[0]-StartPoint[0]);
  //double exSlope;
  double diffSlope;
  
    for(int k = 0; k < z; k++){
      diffSlope = (((StartPoint[1] - MemLines[k].EndPt[1])/(StartPoint[0] - MemLines[k].EndPt[0])) - sentSlope);
      if(abs(diffSlope) <= 0.2*sentSlope){
        return;
      }
    }
    //exSlope = (EndPoint[1]-StartPoint[1])/(EndPoint[0]-StartPoint[0]);
     
  for (int i = 0; i <= MaxLines; i++){
    if(MemLines[i].Taken == false){
      MemLines[i].Taken = true;
      memmove(MemLines[i].StartPt, StartPoint, 2);
      memmove(MemLines[i].EndPt, EndPoint, 2);
      if(Vert){
        MemLines[i].VertSlope = true;
        return;
        }
      if(Horiz){
        MemLines[i].HorizSlope = true;
        return;
        }
      }
      return;
    
    }
  
  
  }

  
// Finished(ish)!!
double *** lineDesig (double ** Pairs, bool Switch, int leng){
  double StartSlope;
  double EndSlope;
  double LineStart[2];
  double LineEnd[2];
  bool Latch = true;
  bool Vertical = false;
  double *** xyarr;
  xyarr = (double ***) malloc(leng);
  int index = 0;
      for (int i = 0; i <= leng; i++){
        if(Latch){
          double tempList[2] = {Pairs[i][0], Pairs[i][1]};
          memcpy(LineStart, tempList, 64);
          delete[] tempList;
          if(Pairs[i][0] != Pairs[i+1][0]){
            StartSlope = (Pairs[i+1][1] - Pairs[i][1]) / (Pairs[i+1][0] - Pairs[i][0]);
            Vertical = false;
            }
          else{
            Vertical = true;
            for(int f = i; f <= leng; f++){
            if(Pairs[f][0] != Pairs[f+1][0]){
              LineEnd[0] = Pairs[f][0];
              LineEnd[1] = Pairs[f][1];
              i = i + f;
              f = leng + 1;
              if(Switch){AddGuideLine(LineStart, LineEnd, true, false);}
              else{
                xyarr[index][0] = LineStart;
                xyarr[index][1] = LineEnd;
                index++;
              }
              Latch = true;
              break;
              }
            }
            }
          }
          else if (Latch){
            EndSlope = (Pairs[i+2][1] - Pairs[i+1][1]) / (Pairs[i+2][0] - Pairs[i+1][0]);
            Latch = false;
            }
         
          if(Pairs[i][0] == Pairs[i+1][0] && Vertical == false){
            LineEnd[0] = Pairs[i][0]; 
            LineEnd[1] = Pairs[i][1];
          
            if(Switch){AddGuideLine(LineStart, LineEnd, false, false);}
            else{
                xyarr[index][0] = LineStart; 
                xyarr[index][1] = LineEnd;
                index++;
              }
          }
          if (abs(StartSlope - EndSlope) >= 0.02 && !Vertical){
            LineEnd[0] = Pairs[i][0]; 
            LineEnd[1] = Pairs[i][1];
            if(StartSlope == 0){
                if(Switch){AddGuideLine(LineStart, LineEnd, false, true);}
                else{
                xyarr[index][0] = LineStart; 
                xyarr[index][1] = LineEnd;
                index++;
              }
            }
            else{
                if(Switch){AddGuideLine(LineStart, LineEnd, false, false);}
                else{
                xyarr[index][0] = LineStart; 
                xyarr[index][1] = LineEnd;
                index++;
              }
            }
            Latch = true;
        }
    if(Latch == false && Vertical == false){
        EndSlope = (Pairs[i+1][1] - Pairs[i][1]) / (Pairs[i+1][0] - Pairs[i][0]);
    }
    count = 0;
  
  
  
  }

  if(Switch){return(xyarr);}
}

  

double * getDist(double ang, bool latch){
  myservo.write(90);
  Serial1.write(trigger, 4);
  //int latch = 0;
  int count = 0;
  double * xylist;
  xylist = (double *) malloc(64);
  while(latch == 0){
    delay(100);
     if(Serial1.available() >= 9){

      for(int i = 0; i < 9; i++){
        Recieved[i] = Serial1.read();
      }

          if(Recieved[0] + (Recieved[1]*255) > 200 && Recieved[0] + (Recieved[1]*255) < 5000){
            xylist = trig_calc(Recieved[0] + (Recieved[1]*255), ang);
            
            return(xylist);
          }
          if(Recieved[0] + (Recieved[1]*255) <= 200){
            Serial.println("The distance is closer than 20 cm");
            return;
          }
  }
  count++;
  if(count >= 5){
  return;
}
}

}

double *** scan(double start, double end, bool latch){
  //Serial.println("Scanning");
  double data[2];
  double prev_angle = 0;
  bool lim = false;
  int limNum = 0;
  double ** tempLines;
  int index = 0;

  tempLines = (double **) malloc(MaxLines*32);
  for(int i = 0; i < 4; i++){
    if(start >= sLP[i][0] && start <= sLP[i][1]){
      start = sLP[i][1] + 0.5;
    }
    if(end >= sLP[i][0] && end <= sLP[i][1]){
      end = sLP[i][1] - 0.5;
    }
  }
  if(end == start){return(-1);}
  
  ang = analogRead(A9);
      if(ang <= 409.6212 && ang > 0){
              angle = d_map(ang, 1, 409.6212, 5.00, 160);
              
            }
            else if(ang > 1023*0.4004 + 1023*0.1992 && ang < 1023){
              angle = d_map(ang, 1023*0.4004 + 1023*0.1992, 1023, 200, 355);
              // Serial.println(angle);
            }

    myservo.write(89);
    int c = 0;
    while(!(angle > start - 0.5 && angle < start + 0.5)){
      Serial.println(ang);
      //Serial.println("Hey");
      if(c < 4){
        myservo.write(89);
      }
      else if(c < 9){
        myservo.write(90);
      }
      else{
        c = 0;
      }
      delay(1);
      ang = analogRead(A9);
      if(ang <= 409.6212 && ang > 0){
              angle = d_map(ang, 1, 409.6212, 5.00, 160);
              
            }
            else if(ang > 1023*0.4004 + 1023*0.1992 && ang < 1023){
              angle = d_map(ang, 1023*0.4004 + 1023*0.1992, 1023, 200, 355);
              // Serial.println(angle);
            }
      c++;
    }
    myservo.write(90);
    prev_angle = angle;
    c = 0;
    bool cringe = false;
    while(!(angle > end - 0.5 && angle < end + 0.5)){
      
      if(c < 4){
        myservo.write(91);
      }
      else if(c < 9){
        myservo.write(90);
      }
      else{
        c = 0;
      }
      delay(1);
      while(!(angle < prev_angle + 0.3 && angle > prev_angle + 0.03)){
        if(angle + 0.3 > end - 0.5 && angle + 0.3 < end + 0.5){
          break;
        }
        if(angle > prev_angle + 0.35){
          prev_angle = angle;
          break;
        }
        
        if(c < 4){
          myservo.write(91);
        }
        else if(c < 9){
          myservo.write(90);
        }
        else{
          c = 0;
        }
        delay(1);
      
        ang = analogRead(A9);
        if(ang <= 409.6212 && ang > 0){
              angle = d_map(ang, 1, 409.6212, 5.00, 160);
              
            }
            else if(ang > 1023*0.4004 + 1023*0.1992 && ang < 1023){
              angle = d_map(ang, 1023*0.4004 + 1023*0.1992, 1023, 200, 355);
              
            }
        c++;
      }
        for(int i = 0; i < 4; i++){
          if(angle >= sLP[i][0] && angle <= sLP[i][1]){
            lim = true;
            limNum = i;
            break;
          }
        }
        if(lim){
          while(angle >= sLP[limNum][0] && angle <= sLP[limNum][1]){
            if(c < 4){
              myservo.write(91);
            }
            else if(c < 9){
              myservo.write(90);
            }
            else{
              c = 0;
            }
            delay(1);
            ang = analogRead(A9);
            if(ang <= 409.6212 && ang > 0){
              angle = d_map(ang, 1, 409.6212, 5.00, 160);
              
            }
            else if(ang > 1023*0.4004 + 1023*0.1992 && ang < 1023){
              angle = d_map(ang, 1023*0.4004 + 1023*0.1992, 1023, 200, 355);
              
            }
            c++;
          }
          lim = false;
        }
        if(ang <= 409.6212 && ang > 0){
              angle = d_map(ang, 1, 409.6212, 5.00, 160);
              
            }
            else if(ang > 1023*0.4004 + 1023*0.1992 && ang < 1023){
              angle = d_map(ang, 1023*0.4004 + 1023*0.1992, 1023, 200, 355);
              
            }
      c++;
    if(c == 4){
      *data = *getDist(ang, latch);
    }
    if(data[0] != -1.0){  
      tempLines[index] = data;
      index++;
    }
    prev_angle = angle;
    c++;
    //Serial.println("Bad bad bad");
    Serial.println("running");
    }
  myservo.write(90);
  if(!latch){
    tempLines = (double **) realloc(tempLines, index+1);
    return(lineDesig(tempLines, latch, index+1));
  }
}


void turn_any(double offset){
  
}

  int * gap_detector(){
    int retLines[MaxLines];
    int i = 0;
    int index = 0;
      while(MemLines[i].Taken == true && MemLines[i+1].Taken == true){
        if(MemLines[i].StartPt[0] < MemLines[i+1].StartPt[0] + 5 && MemLines[i].StartPt[0] > MemLines[i+1].StartPt[0] - 5 && MemLines[i].StartPt[1] < MemLines[i+1].StartPt[1] + 5 && MemLines[i].StartPt[1] > MemLines[i+1].StartPt[1] - 5){
          i++;
        }
        else if(MemLines[i].StartPt[0] < MemLines[i+1].EndPt[0] + 5 && MemLines[i].StartPt[0] > MemLines[i+1].EndPt[0] - 5 && MemLines[i].StartPt[1] < MemLines[i+1].EndPt[1] + 5 && MemLines[i].StartPt[1] > MemLines[i+1].EndPt[1] - 5){
          i++;
        }
        else if(MemLines[i].EndPt[0] < MemLines[i+1].EndPt[0] + 5 && MemLines[i].EndPt[0] > MemLines[i+1].EndPt[0] - 5 && MemLines[i].EndPt[1] < MemLines[i+1].EndPt[1] + 5 && MemLines[i].EndPt[1] > MemLines[i+1].EndPt[1] - 5){
          i++;
        }
        else if(MemLines[i].EndPt[0] < MemLines[i+1].StartPt[0] + 5 && MemLines[i].EndPt[0] > MemLines[i+1].StartPt[0] - 5 && MemLines[i].EndPt[1] < MemLines[i+1].StartPt[1] + 5 && MemLines[i].EndPt[1] > MemLines[i+1].StartPt[1] - 5){
          i++;
        }
        else{
          retLines[index] = 1;
          index++;
          i++;
        }
      }
      return(retLines);
      
      }


double calibrate(){
  double *** lineStEnd;
  double slope = 0.0;
  double relAngle = 0.0;
  bool vert = false;
  lineStEnd = (double ***) malloc(MaxLines);
  lineStEnd = scan(195, 155, false);
  int leng = 0;
  for(int i = 0; i < MaxLines; i++){
    if(lineStEnd[i][0] == 0 || lineStEnd[i][0] == NULL){
      leng = i+1;
      break;
    }
  }
  lineStEnd = (double ***) realloc(lineStEnd, leng);
  double leastVal = 999;
  int index = 0;
  for(int k = 0; k < leng; k++){
    if(leastVal > get_dist_from_pairs(lineStEnd[k][0], lineStEnd[k+1][1])){
      leastVal = get_dist_from_pairs(lineStEnd[k][0], lineStEnd[k+1][1]);
      index = k;
    }
  }
  Serial.println("Got here");
  double ** foundLine;
  foundLine = (double **) malloc(128);
  foundLine = lineStEnd[index];
  //free(lineStEnd);
  if(abs(foundLine[0][0] - foundLine[1][0]) <= 0.5){
    relAngle = 0.0;
    vert = true;
  }
  else if(abs(foundLine[0][1] - foundLine[1][1]) <= 0.5){
    relAngle = 0.0;
    vert = false;
  }
  else{
    relAngle = atan(slope)/57.2958;
  }

  int count = 0;

  for(count = 0; count < 25; count++){
   digitalWrite(12, 0x1); 
   digitalWrite(13, 0x1);
   digitalWrite(9, LOW);
   digitalWrite(8, LOW);
   analogWrite(11, 255);
   analogWrite(3, 255);
   delay(10);
  }
  analogWrite(11, 0);
  analogWrite(3, 0);
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
  

  double relAngle2 = 0.0;
  lineStEnd = scan(200, 160, false);
  leng = 0;
  for(int i = 0; i < MaxLines; i++){
    if(lineStEnd[i][0] == 0 || lineStEnd[i][0] == NULL){
      leng = i+1;
      break;
    }
  }
  lineStEnd = (double ***) realloc(lineStEnd, leng);
  leastVal = 999;
  index = 0;
  for(int k = 0; k < leng; k++){
    if(leastVal > get_dist_from_pairs(lineStEnd[k][0], lineStEnd[k+1][1])){
      leastVal = get_dist_from_pairs(lineStEnd[k][0], lineStEnd[k+1][1]);
      index = k;
    }
  }
  foundLine = lineStEnd[index];
  free(lineStEnd);
  if(abs(foundLine[0][0] - foundLine[1][0]) <= 0.5){
    relAngle2 = 0.0;
    vert = true;
  }
  else if(abs(foundLine[0][1] - foundLine[1][1]) <= 0.5){
    relAngle2 = 0.0;
    vert = false;
  }
  else{
    relAngle2 = atan(slope)/57.2958;
  }

  double angDiff = abs(relAngle2 - relAngle);
  nineDelay = (90/angDiff)*0.25;

}


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial1.begin(115200);

Serial1.write(outputOn, 5);
Serial1.write(format, 5);
Serial1.write(setTrigger, 6);



//pinMode(A9, INPUT);

myservo.attach(7);

myservo.write(90);

pinMode(12, OUTPUT);
pinMode(13, OUTPUT);
pinMode(9, OUTPUT);
pinMode(8, OUTPUT);
pinMode(10, INPUT);



}

bool check = false;

void loop() {
  if(digitalRead(10) == 0x1 && !check){
    delay(5000);
    calibrate();
    check = true;
  }
  //gap_detector(xcoords, ycoords0);
}
