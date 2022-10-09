#include <Math.h>
#include <stdlib.h>
#include <wire.h>
int drivedist = 0;
int MaxLines = 100;
double CollisionBufferSize = 15; // in cm
double posx = 0.0;
double posy = 0.0;
double heading = 0.0;
int list_count = 0;
int sample_size = 300;
double ** coords{300};
const int TF_LunaADRS = 0x10;


/*
 while((0.0000004744*(ang*ang*ang))-(0.0007449*(ang*ang))+(0.6635*ang)-27.21 > 180.3 || (0.0000004744*(ang*ang*ang))-(0.0007449*(ang*ang))+(0.6635*ang)-27.21 < 179.7){
      ang = analogRead(A4);
      if((0.0000004744*(ang*ang*ang))-(0.0007449*(ang*ang))+(0.6635*ang)-27.21 > 180.3){
        digitalWrite(12, HIGH); //probably needs to be changed later
        analogWrite(3, 128);
        }
        if((0.0000004744*(ang*ang*ang))-(0.0007449*(ang*ang))+(0.6635*ang)-27.21 < 179.7){
        digitalWrite(12, LOW); //probably needs to be changed later
        analogWrite(3, 128);
        }
       delay(5);
      }
 */


class GuideLines {
  public:
    bool Taken = false;  
    double StartPt = 0.0;
    double EndPt = 0.0;
    bool SlopeYN = true;
    bool VertSlope = false;
    bool HorizSlope = false;
    double altSartPt = 0.0;
    double altEndPt = 0.0;
}

class NavLines {
  public:
    bool Taken = false;  
    double StartPt = 0.0;
    double EndPt = 0.0;
    bool SlopeYN = true;
    bool VertSlope = false;
    bool HorizSlope = false;
    double altSartPt = 0.0;
    double altEndPt = 0.0;
}

GuideLines MemLines[MaxLines];

void AddGuideLine(double * StartPoint[2], double * EndPoint[2], bool Vert, bool Horiz){
  for (i = 0; i <= MaxLines; i++;){
    if(MemLines[i].Taken == false){
      MemLines[i].Taken = true;
      memmove(MemLines[i].Startpt, StartPoint, 2);
      memmove(MemLines[i].Endpt, EndPoint, 2);
      if(Vert){
        MemLines[i].VertSlope = true;
        return();
        }
      if(Horiz){
        MemLines[i].HorizSlope = true;
        return();
        }
      }
      return();
    
    }
  
  
  }
// Finished(ish)!!
void LineDesig (double ** Pairs{}, bool Switch){
  int leng = sizeof(Pairs) / 64;
  double StartSlope;
  double EndSlope;
  double * LineStart[2];
  double * LineEnd[2];
  bool Latch = true;
  bool Vertical = false;
      for (i = 0; i <= leng; i++;){
        if(Latch){
          LineStart = [Pairs[i][0], Pairs[i][1]];
          if(Pairs[i][0] != Pairs[i+1][1]){
            StartSlope = (Pairs[i+1][1] - Pairs[i][1]) / (Pairs[i+1][0] - Pairs[i][0]);
            Vertical = false;
            }
          else{
            Vertical = true;
            }
          }
        if(Pairs[i][0] == Pairs[i+1][0] && Vertical == true){
          for(f = i; f <= leng; f++;){
            if(Pairs[f][0] != Pairs[f+1][0]){
              LineEnd = [Pairs[f][0], Pairs[f][1]];
              i = i + f;
              f = leng + 1;
              AddGuideLine(LineStart, LineEnd, true, false);
              Latch = true;
              }
            }
          
          }
          else if (Latch){
            EndSlope = (Pairs[i+2][1] - Pairs[i+1][1]) / (Pairs[i+2][0] - Pairs[i+1][0]);
            Latch = false;
            }
         
          if(Pairs[i][0] == Pairs[i+1][0] && Vertical == false){
            LineEnd = [Pairs[i][0], Pairs[i][1]];
            AddGuideLine(LineStart, LineEnd, false, false);
            }
          if (abs(StartSlope - EndSlope) >= 0.02 && Vertical == false){
            LineEnd = [Pairs[i][0], Pairs[i][1]];
            if(StartSlope == 0){
                AddGuideLine(LineStart, LineEnd, false, true);
              }
            else{
                AddGuideLine(LineStart, LineEnd, false, false;)
              }
            }
           if(Latch == false && Vertical == false){
            EndSlope = (Pairs[i+1][1] - Pairs[i][1]) / (Pairs[i+1][0] - Pairs[i][0]);
            }
        }
  
  
  }
// unfinshed
void scan(bool guide){
  double ang = 0.0;
  double angl = 0.0;
  double result[2];
  double cm = 0;
  int count = 0;
  ang = analogRead(A4);
  angl = 0.0000004744*(ang*ang*ang))-(0.0007449*(ang*ang))+(0.6635*ang)-27.21;
  if(guide){
    angl = abs(angl - heading);
  }
  Serial.println("got here");
  // digitalWrite(9, HIGH); whatever makes activates the scanning motor
  while(count <= sample_size){
    ang = analogRead(A4);
    angl = 0.0000004744*(ang*ang*ang))-(0.0007449*(ang*ang))+(0.6635*ang)-27.21;
    if(guide){
      angl = abs(angl - heading);
    }
    //cm = take measurement
    trig_calc
    } 
  }

  double trig_calc (double dist, double angl){
    rads = angl/57.2957795;
    double x = cos(rads)*dist;
    double y = sin(rads)*dist;
    if (angl <= 90){
      
      }
      }
    double xylist[2];
    xylist = [0, 0];
    xylist[0] = x;
    xylist[1] = y;
    return(xylist); 
    }

// thhis will be removed at some point
  void gap_detector(double xarray[4096], double yarray[4096]){
      int arraynumeral = 0;
      double now;
      double future;
      double difference;
      
      for(j = 0, j <= 4095, j+= 1){
        now = xcoords[j];
        future = xcoords[j+1];
        difference = now - future;
        if (abs(difference) < 100 && abs(difference) > 10){
            xxgapcoords[arraynumeral] = (now+future)/2;
            yxgapcoords[arraynumeral] = ycoords0[j];
            arraynumeral += 1;
          }
        if (xcoords[j] == 0){
          j = 5000;
          }
        }
        arraynumeral = 0
        for(j = 0, j <= 4095, j+= 1){
        now = ycoords[j];
        future = ycoords[j+1];
        difference = now - future;
        if (abs(difference) < 100 && abs(difference) > 10){
            yygapcoords[arraynumeral] = (now+future)/2;
            xygapcoords[arraynumeral] = xcoords[j];
            arraynumeral += 1;
          }
        if(xcoords[j] == 0){
          j = 5000;
          }
        }
      
      }


void setup() {
  // put your setup code here, to run once:
 posx = 0.0;
 posy = 0.0;
 orient = 0;
pinMode(12, OUTPUT);
pinMode(9, OUTPUT);
pinMode(7, OUTPUT);
pinMode(8, INPUT)
digitalWrite(7, LOW);
analogWrite(3, 0);
digitalWrite(9, HIGH);
// once again this is outdated
float ree;
delay(10000);
ree = analogRead(A1);
if ((0.0000004744*(ree*ree*ree))-(0.0007449*(ree*ree))+(0.6635*ree)-27.21 > 180.3 || (0.0000004744*(ree*ree*ree))-(0.0007449*(ree*ree))+(0.6635*ree)-27.21 < 179.7){
    digitalWrite(9, LOW);
    while((0.0000004744*(ree*ree*ree))-(0.0007449*(ree*ree))+(0.6635*ree)-27.21 > 180.3 || (0.0000004744*(ree*ree*ree))-(0.0007449*(ree*ree))+(0.6635*ree)-27.21 < 179.7){
      ree = analogRead(A1);
      if((0.0000004744*(ree*ree*ree))-(0.0007449*(ree*ree))+(0.6635*ree)-27.21 > 180.3){
        digitalWrite(12, HIGH); //probably needs to be changed later
        analogWrite(3, 64);
        }
        if((0.0000004744*(ree*ree*ree))-(0.0007449*(ree*ree))+(0.6635*ree)-27.21 < 179.7){
        digitalWrite(12, LOW); //probably needs to be changed later
        analogWrite(3, 64);
        }
       delay(10);
      }
    analogWrite(3, 0);
    digitalWrite(9, HIGH);
  }

}


// Almost everything in loop is outdated

void loop() {
  scan();
  for(i = 0, i <= 4095, i += 1){
    if(xcoords[i] > posx - 20){
      //turn right until 90 deg from wall and drive
      }
    if(xcoords[i] < posx + 20){
      //turn left until 90 deg from wall and drive
      }
    if(ycoords[i] < posy + 20){
      //turn left until 90 deg from wall and drive
    }
    if(ycoords[i] > posy - 20){
      //turn left until 90 deg from wall and drive
      }
   if(xcoords[i] == 0){
    i = 5000;
    }
    }
    gap_detector(xcoords, ycoords0);
    double dist = 0.0;
    double protodistx = 0.0;
    double protodisty = 0.0;
    double coordx = 0.0;
    double coordy = 0.0;
    boolean alignment = false;
  for(i = 0, i<=256, i += 1){
    protodistx = (abs(posx - xxgapcoords[i]) + abs(posy - yxgapcoords[i]));
    protodisty = (abs(posx - xygapcoords[i]) + abs(posy - yygapcoords[i]));
    if(xxgapcoords[i] == 0 || xygapcoords[i] == 0){
      i = 257;
      break();
      }

    if(protodistx < protodisty){
      if(dist > protodistx){
        dist = protodistx;
        coordx = xxgapcoords[i];
        coordy = yxgapcoords[i];
        alignment = false;
        }
      }
     if(protodistx > protodisty){
      if(dist > protodisty){
        dist = protodisty;
        coordx = xygapcoords[i];
        coordy = yygapcoords[i];
        alignment = true;
        }
      }
    }
    double distx = posx - coordx
    double disty = posy - coordy
    if (alignment){
      if (orient == 1){
        if (coordx > posx){
         drive(2, orient);
          }
          if (coordx < posx){
          drive(3, orient);
          }
         
        }
      if(orient == 2){
        if (coordx < posx){
          drive(2, orient);
          drive(2, orient);
          }
        }
      if(orient == 4){
        if(coordx > posx){
          drive(3, orient);
          dirve(3, orient);
          }
      }
      if(orient == 3){
        if(coordx > posx){
          drive(3, orient);
          }
         if(coordx < posx){
          drive(2, orient);
          }
        }
        
    }
    if(alignment == false){
      if(orient == 1){
        if(coordy < posx){
          drive(2, orient);
          drive(2, orient);
          }
        }
      if(orient == 2){
        if(coordy > posy){
          drive(3, orient);
          }
        if(coordy < posy){
          drive(2, orient);
          }
        }
      if(orient == 4){
        if(coordy > posy){
          drive(2, orient);
          }
        if(coordy < posy){
          dirve(3, orient);
          }
        }
      if(orient == 3){
        if(coordy > posy){
          drive(2, orient);
          dirve(2, orient);
          }
        }
      }
      if(alignment){
        drivedist = (abs(posx - distx));
        drive(1, orient);
        if(coordy > posy && orient == 2){
          drive(3, orient);
          }
        if(coordy > posy && orient == 4){
          drive(2, orient);
          }
        if(coordy < posy && orient == 4){
          drive(3, orient);
          }
        if(coordy < posy && orient == 2){
          drive(2, orient);
          }
        drivedist = (abs(posy - coordy));
        drive(1, orient);
          }
        if(alignment == false){
        drivedist = (abs(posy - disty));
        drive(1, orient);
        if(coordx > posx && orient == 1){
          drive(2, orient);
          }
        if(coordx > posx && orient == 3){
          drive(3, orient);
          }
        if(coordx < posx && orient == 1){
          drive(3, orient);
          }
        if(coordx < posx && orient == 3){
          drive(2, orient);
        drivedist = (abs(posy - coordy));
        drive(1, orient);
          }
     }   
}
