// GeoSolver.cpp
//inplemented mathematical engine to solve several geodetic problems

#include "GeoSolver.h"
#include "math.h"
using namespace std;

namespace GeoSol {
    
    int GeoFuncs::R = 6371; //Earth radius 
    double GeoFuncs::PI = 3.1415927; //PI value
    double GeoFuncs::f = 1 / 298.257223563; //flattening of ellipsoid
    
    //convert radians to degrees 
    double GeoFuncs::rad2deg(double rad) {
        return rad * 180 / PI;
    }

    //convert degrees to radians
    double GeoFuncs::deg2rad(double deg) {
        return deg * PI / 180;
    }
    
    //compute direct distance on the plane using simple Pythagorean theorem
    double GeoFuncs::directDistance(double x1, double y1, double x2, double y2) {
        return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    }
    
    //compute azimuths on the plane using trigonometric functions
    double GeoFuncs::directAngle(double x1, double y1, double x2, double y2) {
        double b;
        b = directDistance(x1, y1, x2, y2);
        if (x1 <= x2 && y2 >= y1)
            return asin(abs(x1 - x2) / b);
        else if (x1 < x2 && y2 < y1)
            return PI - asin(abs(x1 - x2) / b);
        else if (x1 > x2 && y2 < y1)
            return PI + asin(abs(x1 - x2) / b);
        else if (x1 > x2 && y2 > y1)
            return 1.5 * PI + asin(abs(y1 - y2) / b);
        else
            return 0;
    }
    
    //compute distance between two points on sphere with radius R using solid geometry rules
    double GeoFuncs::inverseDistanceGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon) {
        return acos(sin(deg2rad(p1Lat)) * sin(deg2rad(p2Lat)) + cos(deg2rad(p1Lat)) * cos(deg2rad(p2Lat)) * cos(deg2rad(p1Lon - p2Lon))) * R;
    }
    
    //compute azimuth between two points on sphere using Vincenty's formula 
    double GeoFuncs::inverseAzimuthGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon) {
        double U1, U2, L, C, lambda, sintheta, costheta, theta, sinalfa, cos2alfa, cos2theta;
        U1 = atan((1 - f) * tan(deg2rad(p1Lat))); //reduced latitude (latitude on the auxiliary sphere)
        U2 = atan((1 - f) * tan(deg2rad(p2Lat))); //reduced latitude (latitude on the auxiliary sphere)
        L = p2Lon - p1Lon; //difference in longitude
        lambda = L; //starting value for lambda
        
        //wait while lamda converges to 10^(-8) which is sufficiently enough accuracy for us
        while (abs(lambda) >= 0.00000001) {
            sintheta = sqrt(pow(cos(U2) * sin(lambda), 2) + pow(cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(lambda), 2));
            costheta = sin(U1) * sin(U2) + cos(U1) * cos(U2) * cos(lambda);
            theta = atan2(sintheta, costheta);
            sinalfa = cos(U1) * cos(U2) * sin(lambda) / sin(theta);
            cos2alfa = 1 - pow(sinalfa, 2);
            cos2theta = costheta - 2 * sin(U1) * sin(U2) / cos2alfa;
            C = f / 16 * cos2alfa * (4 + f * (4 - 3 * cos2alfa));
            lambda = L + (1 - C) * f * sinalfa * (theta + C * sintheta * (cos2theta + C * costheta * (1 * pow(cos2theta, 2) - 1)));
        }
        
        //and then use this lambda in formula 
        return rad2deg(atan2(cos(U2) * sin(lambda), (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(lambda)))); 
    }
    
    //compute latitude of point on sphere from direct problem using solid geometry rules 
    double GeoFuncs::directLatGP(double p1Lat, double p1Lon, double angle, double dist)
    {
        double relDist, relp1Lat;
        relDist = dist / R;
        relp1Lat = deg2rad(p1Lat);
        angle = deg2rad(angle);
        return rad2deg(asin(sin(relp1Lat) * cos(relDist) + cos(relp1Lat) * sin(relDist) * cos(angle)));
    }
    
    //compute longitude of point on sphere from direct problem using solid geometry rules
    double GeoFuncs::directLonGP(double p1Lat, double p1Lon, double angle, double dist)
    {
        double relDist, relp1Lat;
        relDist = dist / R;
        relp1Lat = deg2rad(p1Lat);
        angle = deg2rad(angle);
        return p1Lon + rad2deg(atan2(sin(angle) * sin(relDist) * cos(relp1Lat), 
                    cos(relDist) - sin(relp1Lat) * sin(relp1Lat) * cos(relDist) + cos(relp1Lat) * sin(relDist) * cos(angle)));
    }
    
    //compute latitude of point on sphere from polar problem using solid geometry rules 
    double GeoFuncs::polarLatGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon, double angle, double dist)
    {
        angle += inverseAzimuthGP(p1Lat, p1Lon, p2Lat, p2Lon);
        return directLatGP(p1Lat, p1Lon, angle, dist);
    }

    //compute longitude of point on sphere from polar problem using solid geometry rules 
    double GeoFuncs::polarLonGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon, double angle, double dist)
    {
        angle += inverseAzimuthGP(p1Lat, p1Lon, p2Lat, p2Lon);
        return directLonGP(p1Lat, p1Lon, angle, dist);
    }
}