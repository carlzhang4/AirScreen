package com.dji.simulatorDemo;

public class CoordinateConversion {

    public Point basePoint;
    public Coordinate current_point;
    private static final double MACRO_AXIS = 6378137; // 赤道圆的平均半径
    private static final double MINOR_AXIS = 6356752; // 半短轴的长度，地球两极距离的一半
    private static final double R = 6371000;

    public CoordinateConversion(double longitude, double latitude, float altitude){ //init
        basePoint = new Point();
        basePoint.altitude = altitude;
        basePoint.latitude = latitude;
        basePoint.longitude = longitude;
    }

    public Coordinate convert(double longitude, double latitude, float altitude, double yaw){

        current_point = new Coordinate();
        double angle_y = latitude - basePoint.latitude;
        double angle_x = (longitude - basePoint.longitude);
        double R_small = R * Math.cos(Math.toRadians(latitude));
        current_point.y = R * Math.toRadians(angle_y);
        current_point.x = R_small * Math.toRadians(angle_x);
        current_point.z = altitude - basePoint.altitude;
        current_point.o = yaw - 0;
        return current_point;
    }

    public class Coordinate{
        public double x,y;
        public float z;
        public double o;

        public Coordinate(){

        }
        public Coordinate(double azimuth, double pitch, double radius){
            pitch = -pitch;
            double r = radius * Math.cos(Math.toRadians(pitch));
            z = (float)(radius * Math.sin(Math.toRadians(pitch)));
            x = r * Math.sin(Math.toRadians(azimuth));
            y = r * Math.cos(Math.toRadians(azimuth));
        }
    }

    public class Point{
        public double latitude;
        public double longitude;
        public float altitude;
        public float yaw;
    }
}

