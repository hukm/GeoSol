#include "mbed.h"
#include "rtos.h"
#include "C12832.h"
#include "TinyGPS.h"
#include <vector>
#include <string> 
#include "math.h"
using namespace std;

C12832 lcd(D11, D13, D12, D7, D10);
DigitalIn Up(A2);
DigitalIn Down(A3);
DigitalIn Left(A5);
DigitalIn Right(A4);
DigitalIn Click(D4);
PwmOut r (D5);
PwmOut g (D8);
PwmOut b (D9);
AnalogIn pot1 (A0);
AnalogIn pot2 (A1);

TinyGPS gpsr;
Serial serial_gps(D1, D0); //tx,rx
char* joystickPos = "CENTRE";
char latString[10] = "";
char lonString[10] = "";
int menuItem = 0;
int menuPosition = 0;
int menuItemCount = 4;
vector<int> menuPositionCount(menuItemCount);

double lat, lon;
unsigned long age;
double potDist, potAngle;
bool checked;

class geodeticSolver{
private:
const double PI = 3.1415927;

public:
const int R = 6371;
double rad2deg(double rad){return rad*180/PI;}
double deg2rad(double deg){return deg*PI/180;}
double directDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));   
}
double directAngle(double x1, double y1, double x2, double y2)
{
    double b;
    b = directDistance(x1,y1,x2,y2);
    if (x1 <= x2 && y2 >= y1)
        return asin(abs(x1 - x2)/b);
    else if (x1 < x2 && y2 < y1)
        return PI - asin(abs(x1 - x2)/b);
    else if (x1 > x2 && y2 < y1)
        return PI + asin(abs(x1 - x2)/b);
    else if (x1 > x2 && y2 > y1)
        return 1.5*PI + asin(abs(y1 - y2)/b);
    else 
        return 0;
}

};

struct problem
{
    double p1Lat, p1Lon, p2Lat, p2Lon, p3Lat, p3Lon;
    double dist, angle;
    bool solved;
} GP[3];                                       //We solve 3 different geodetic problems
                                                            //0 - inverse geodetic problem
                                                            //1 - direct geodetic problem
                                                            //2 - polar serif problem

void solveInverseGP()
{
    geodeticSolver g;
    
    GP[0].dist = acos(sin(g.deg2rad(GP[0].p1Lat))*sin(g.deg2rad(GP[0].p2Lat)) + cos(g.deg2rad(GP[0].p1Lat))*cos(g.deg2rad(GP[0].p2Lat))*cos(g.deg2rad(GP[0].p1Lon-GP[0].p2Lon))*g.R);
    GP[0].angle = g.rad2deg(g.directAngle(GP[0].p1Lat,GP[0].p2Lat,GP[0].p1Lon,GP[0].p2Lon));
}
void solveDirectGP()
{
    geodeticSolver g;
    
    double relDist, angle, relp1Lat;
    relDist = GP[1].dist / g.R;
    relp1Lat = g.deg2rad(GP[1].p1Lat);
    angle = g.deg2rad(GP[1].angle);
    GP[1].p2Lat = g.rad2deg(asin(sin(relp1Lat)*cos(relDist) + cos(relp1Lat)*sin(relDist)*cos(angle)));
    GP[1].p2Lon = GP[1].p1Lon + g.rad2deg(atan2(sin(angle)*sin(relDist)*cos(relp1Lat), cos(relDist)-sin(relp1Lat)*sin(g.deg2rad(GP[1].p2Lat))));
}
void solvePolarSP()
{   
    geodeticSolver g;
    
    double relDist, angle, relp1Lat;
    angle = g.directAngle(GP[2].p1Lat,GP[2].p2Lat,GP[2].p1Lon,GP[2].p2Lon);
    angle += g.deg2rad(GP[2].angle); //new angle to calculate direct problem 
    relDist = GP[2].dist / g.R;
    relp1Lat = g.deg2rad(GP[2].p1Lat);
    GP[2].p3Lat = g.rad2deg(asin(sin(relp1Lat)*cos(relDist) + cos(relp1Lat)*sin(relDist)*cos(angle)));
    GP[2].p3Lon = GP[2].p1Lon + g.rad2deg(atan2(sin(angle)*sin(relDist)*cos(relp1Lat), cos(relDist)-sin(relp1Lat)*sin(g.deg2rad(GP[2].p3Lat))));
}

void updateValue()
{
    //updating of Inverse GP parameters    
    if (menuItem == 1 && menuPosition == 1){
        GP[0].p1Lat = lat;
        GP[0].p1Lon = lon;}
    else if (menuItem == 1 && menuPosition == 2){
        GP[0].p2Lat = lat;
        GP[0].p2Lon = lon;}
        
    //updating of Direct GP parameters 
    else if (menuItem == 2 && menuPosition == 1){
        GP[1].p1Lat = lat;
        GP[1].p1Lon = lon;}
    else if (menuItem == 2 && menuPosition == 2){
        GP[1].dist = potDist;}
    else if (menuItem == 2 && menuPosition == 3){
        GP[1].angle = potAngle;}
        
    //updating of Polar serif problem parameters 
    else if (menuItem == 3 && menuPosition == 1){
        GP[2].p1Lat = lat;
        GP[2].p1Lon = lon;}
    else if (menuItem == 3 && menuPosition == 2){
        GP[2].p2Lat = lat;
        GP[2].p2Lon = lon;}
    else if (menuItem == 3 && menuPosition == 3){
        GP[2].dist = potDist;}
    else if (menuItem == 3 && menuPosition == 4){
        GP[2].angle = potAngle;}
        
    // Recalculate specific problem if entered parameters are enough for solution            
    if (menuItem == 1 && (GP[0].p1Lat != 0 || GP[0].p1Lon != 0) && (GP[0].p2Lat != 0 || GP[0].p2Lon != 0)){
        GP[0].solved = true;
        solveInverseGP();
    }
    else if (menuItem == 2 && (GP[1].p1Lat != 0 || GP[1].p1Lon != 0)){
        GP[1].solved = true;
        solveDirectGP();
    }
    else if (menuItem == 3 && (GP[2].p1Lat != 0 || GP[2].p1Lon != 0) && (GP[2].p2Lat != 0 || GP[2].p2Lon != 0)){
        GP[2].solved = true;
        solvePolarSP();
    }
}

void printMenu(int menuItem, int menuPosition) 
{
    static char last_line1[30] = "", last_line2[30] = "", last_line3[30] = ""; // keep the data which was printed before 
    char line1[30] = "", line2[30] = "", line3[30] = "";                       // and not update display in case lines haven't changed
                                                                               // otherwise display will blink each second on update
    switch(menuItem)
    {
        //Instruction set
        case 0:                     
            switch(menuPosition)
            {
                case 0:
                    sprintf(line1, "       Instructions");      //
                    sprintf(line2, "Scroll down with joystick");
                    sprintf(line3, "to learn more.");
                    break;
                case 1:
                    sprintf(line1, "To change parameter click");
                    sprintf(line2, "with joystick. Click again");
                    sprintf(line3, "for GPS coordinate.");
                    break;
                case 2:
                    sprintf(line1, "Use potentiometer to ");
                    sprintf(line2, "change float values.");
                    sprintf(line3, "");
                    break;   
                case 3: 
                    sprintf(line1, "Device will turn off LED when");
                    sprintf(line2, "GPS satellites will be found");
                    sprintf(latString, "%f", lat);
                    sprintf(lonString, "%f", lon);  
                    strcpy(line3, latString);
                    strcat(line3, "  "); 
                    strcat(line3, lonString);
                    break;
                case 4: 
                    sprintf(line1, "To solve geodetic problems");
                    sprintf(line2, "go up with joystick and then");
                    sprintf(line3, "navigate right or left");
                    break;                               
            }
            break;
        
        //Inverse geodetic problem
        case 1:                     
            switch(menuPosition)
            {
                //Title
                case 0:
                    sprintf(line1, "          Inverse");  
                    sprintf(line2, "      geodetic problem");
                    sprintf(line3, "");
                    break;
                
                //Point 1
                case 1:
                    sprintf(line1, "Point 1");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(latString, "%f", GP[menuItem-1].p1Lat);
                        sprintf(lonString, "%f", GP[menuItem-1].p1Lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(latString, "%f", lat);
                        sprintf(lonString, "%f", lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                    }
                    break;
                
                //Point 2
                case 2:
                    sprintf(line1, "Point 2");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(latString, "%f", GP[menuItem-1].p2Lat);
                        sprintf(lonString, "%f", GP[menuItem-1].p2Lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(latString, "%f", lat);
                        sprintf(lonString, "%f", lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                    }
                    break;        
                
                //Distance
                case 3:
                    sprintf(line1, "Distance");
                    sprintf(line2, "between two points");
                    sprintf(line3, "%f", GP[menuItem-1].dist);
                    break;
                
                //Angle
                case 4:
                    sprintf(line1, "Angle");
                    sprintf(line2, "between two points");
                    sprintf(line3, "%f", GP[menuItem-1].angle);
                    break;                        
            }
            break;
        
        //Direct geodetic problem
        case 2:                     
            switch(menuPosition)
            {
                //Title
                case 0:
                    sprintf(line1, "          Direct");  //
                    sprintf(line2, "      geodetic problem");
                    sprintf(line3, "");
                    break;
                
                //Point 1
                case 1:
                    sprintf(line1, "Point 1");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(latString, "%f", GP[menuItem-1].p1Lat);
                        sprintf(lonString, "%f", GP[menuItem-1].p1Lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(latString, "%f", lat);
                        sprintf(lonString, "%f", lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    break;
                
                //Distance
                case 2:
                    sprintf(line1, "Distance");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(line3, "%0.3f", GP[menuItem-1].dist);
                    }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(line3, "%0.3f", potDist);
                    }
                    break; 
                
                //Angle       
                case 3:
                    sprintf(line1, "Angle");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(line3, "%0.3f", GP[menuItem-1].angle);
                    }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(line3, "%0.3f", potAngle);
                    }
                    break;
                    
                //Point 2
                case 4:
                    sprintf(line1, "Point 2");
                    sprintf(latString, "%f", GP[menuItem-1].p2Lat);
                    sprintf(lonString, "%f", GP[menuItem-1].p2Lon);  
                    strcpy(line2, latString);
                    strcat(line2, "  "); 
                    strcat(line2, lonString);
                    break;                      
            }
            break;
        //Polar serif problem
        case 3:                     
            switch(menuPosition)
            {
                
                //Title
                case 0:
                    sprintf(line1, "           Polar");  
                    sprintf(line2, "        serif problem");
                    sprintf(line3, "");
                    break;
                
                //Point 1
                case 1:
                    sprintf(line1, "Point 1");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(latString, "%f", GP[menuItem-1].p1Lat);
                        sprintf(lonString, "%f", GP[menuItem-1].p1Lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(latString, "%f", lat);
                        sprintf(lonString, "%f", lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    break;
                    
                //Point 2    
                case 2:
                    sprintf(line1, "Point 2");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(latString, "%f", GP[menuItem-1].p2Lat);
                        sprintf(lonString, "%f", GP[menuItem-1].p2Lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                        }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(latString, "%f", lat);
                        sprintf(lonString, "%f", lon);  
                        strcpy(line3, latString);
                        strcat(line3, "  "); 
                        strcat(line3, lonString);
                    }
                    break;
                    
                //Distance        
                case 3:
                    sprintf(line1, "Distance");
                    if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(line3, "%0.3f", GP[menuItem-1].dist);
                    }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(line3, "%0.3f", potDist);
                    }
                    break;
                
                //Angle
                case 4:
                    sprintf(line1, "Angle");
                   if (!checked){
                        sprintf(line2, "Click to change parameter");
                        sprintf(line3, "%0.3f", GP[menuItem-1].angle);
                    }
                    else{
                        sprintf(line2, "Click to save parameter");
                        sprintf(line3, "%0.3f", potAngle);
                    }
                    break; 
                    
                //Point 3  
                case 5:
                    sprintf(line1, "Point 3");
                    sprintf(latString, "%f", GP[menuItem-1].p3Lat);
                    sprintf(lonString, "%f", GP[menuItem-1].p3Lon);  
                    strcpy(line2, latString);
                    strcat(line2, "  "); 
                    strcat(line2, lonString);
                    break;                  
            }
            break;        
    }
    
    if (strcmp(line1, last_line1) != 0 || strcmp(line2, last_line2) != 0 || strcmp(line3, last_line3) != 0) //here the comparison is held
    {                                                                                                       //and displaying of data as well
        lcd.cls(); 
        lcd.locate(0,0);
        lcd.printf(line1);
        strncpy(last_line1, line1, sizeof(last_line1));

        lcd.locate(0,10);
        lcd.printf(line2);
        strncpy(last_line2, line2, sizeof(last_line2));
        
        lcd.locate(0,20);
        lcd.printf(line3);
        strncpy(last_line3, line3, sizeof(last_line3));
    }
}


void setMenu()
{
    if (Down)
    {
        joystickPos = "DOWN";
        checked = false;
        if (menuPosition >= 0 && menuPosition < menuPositionCount[menuItem] - 1){
            menuPosition++;
            printMenu(menuItem, menuPosition);
            }
    } 
    else if (Left)
    {
        joystickPos = "LEFT";
        checked = false;
         if (menuItem <= menuItemCount && menuItem > 0 && menuPosition == 0){
            menuItem--;
            printMenu(menuItem, menuPosition);
            }
    }
    else if (Click)
    {
        joystickPos = "CLICK";
        if (checked)
        {
            checked = false;
            updateValue();
        }
        else
            checked = true;
    }
    else if (Up)
    {
        joystickPos = "UP";
        checked = false;
        if (menuPosition <= menuPositionCount[menuItem] && menuPosition > 0){
            menuPosition--;
            printMenu(menuItem, menuPosition);
            }
    }
    else if (Right)
    {
        joystickPos = "RIGHT";
        checked = false;
        if (menuItem >= 0 && menuItem < menuItemCount - 1 && menuPosition == 0){
            menuItem++;
            printMenu(menuItem, menuPosition);
            }
    }
    else
        joystickPos = "CENTRE";
}

void menu_loop(void const *args)
{
    int count = 0;
    while(true)
    {
        setMenu();               //Response on button press each 100 milliseconds
        if (++count % 2 == 0){
            printMenu(menuItem, menuPosition); //Update menu each half a second in order to update values
            count = 0;
        }
        Thread::wait(100);       
    } 
}

void ledOff()
{
    r = 1.0; g = 1.0; b = 1.0;    // 1 is off, 0 is full brightness
}


int main()
{
    menuPositionCount[0] = 5;
    menuPositionCount[1] = 5;
    menuPositionCount[2] = 5;
    menuPositionCount[3] = 6;
    for (int i=0; i<2; i++)
    {
        GP[i].p1Lat = 0;   
        GP[i].p1Lon = 0;
        GP[i].p2Lat = 0;   
        GP[i].p2Lon = 0;
        GP[i].p3Lat = 0;   
        GP[i].p3Lon = 0;
        GP[i].dist = 0;   
        GP[i].angle = 0;
        GP[i].solved = false; 
    };
    checked = false;
    int j=0;
    lcd.cls();
    while (j<128) {
        lcd.locate(j,0);
        lcd.pixel(j,0,1);
        lcd.locate(0,50);
        lcd.printf(" ");
        j++;    
        wait(0.01);
    }
    lcd.locate(32,12);
    lcd.printf("GeoSol device");
    
    serial_gps.baud(9600);
    wait(1.0);
    Thread menu_thread(menu_loop);  
    
    while (true){
    if(serial_gps.readable())
    {
        char c = serial_gps.getc();
        bool gps_available = gpsr.encode(c);
        if(gps_available)
        {
            ledOff();
            (void)gpsr.f_get_position(&lat, &lon, &age);
            int ip, fp;
            ip = (int)(pot1*100);
            fp = (int)(pot2*1000);
            potDist = ip + fp*0.001;
            ip = (int)(pot1*360);
            potAngle = ip + fp*0.001;
        }
    }
    }
}