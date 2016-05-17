// GeoSolver.h

namespace GeoSol
{
    class GeoFuncs
    {
    public:
            
        //PI value
        static double PI;
        
        //Earth radius
        static int R;
        
        //Flattening of ellipsoid
        static double f;
        
        //Converts degree into radians
        static double rad2deg(double rad);
        
        //Converts radians into degree
        static double deg2rad(double deg);
        
        //Returns direct distance between points A(x1,y1) and B(x2,y2)
        static double directDistance(double x1, double y1, double x2, double y2);
        
        //Returns azimuth between points A(x1,y1) and B(x2,y2)
        static double directAngle(double x1, double y1, double x2, double y2);
        
        //Returns distance between points A(p1Lat, p1Lon) and B(p2Lat, p2Lon)
        static double inverseDistanceGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon);
        
        //Returns azimuth between points A(p1Lat, p1Lon) and B(p2Lat, p2Lon)
        static double inverseAzimuthGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon);
        
        //Returns latitude of desired point in result of direct geodesic problem
        static double directLatGP(double p1Lat, double p1Lon, double angle, double dist);
        
        //Returns longitude of desired point in result of direct geodesic problem
        static double directLonGP(double p1Lat, double p1Lon, double angle, double dist);
        
        //Returns latitude of desired point in result of Polar serif problem
        static double polarLatGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon, double angle, double dist);
        
        //Returns longitude of desired point in result of Polar serif problem
        static double polarLonGP(double p1Lat, double p1Lon, double p2Lat, double p2Lon, double angle, double dist);
    };
} 