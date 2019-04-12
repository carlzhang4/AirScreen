package com.dji.simulatorDemo;

public class CoordinateConversion {


    public static Coordinate current_coordinate = new Coordinate();
    public static Coordinate goal_coordinate = new Coordinate();
    private static final double MACRO_AXIS = 6378137; // 赤道圆的平均半径
    private static final double MINOR_AXIS = 6356752; // 半短轴的长度，地球两极距离的一半
    private static final double R = 6371000;
    public static Point basePoint = new Point();

    static public void setBasePoint(double longitude, double latitude, float altitude){//init
        basePoint.altitude = altitude;
        basePoint.latitude = latitude;
        basePoint.longitude = longitude;
        basePoint.yaw = 0;
    }

    static public void update_current_coordinate(double longitude, double latitude, float altitude, double yaw){
        Tool.save_type_two(longitude,latitude,altitude,yaw);
        double angle_y = latitude - basePoint.latitude;
        double angle_x = (longitude - basePoint.longitude);
        double R_small = R * Math.cos(Math.toRadians(latitude));
        current_coordinate.y = R * Math.toRadians(angle_y);
        current_coordinate.x = R_small * Math.toRadians(angle_x);
        current_coordinate.z = altitude - basePoint.altitude;
        current_coordinate.o = yaw - 0;
    }

    static public void set_goal_coordinate(double x,double y,float z,float o){
        goal_coordinate.x = x;
        goal_coordinate.y = y;
        goal_coordinate.z = z;
        goal_coordinate.o = o;
    }

    static public void update_goal_coordinate(double azimuth, double pitch, double radius){
        pitch = -pitch;
        double r = radius * Math.cos(Math.toRadians(pitch));
        goal_coordinate.z = (float)(radius * Math.sin(Math.toRadians(pitch)));
        goal_coordinate.x = r * Math.sin(Math.toRadians(azimuth));
        goal_coordinate.y = r * Math.cos(Math.toRadians(azimuth));
        goal_coordinate.o = 0;
    }

    static public class Coordinate{
        public double x,y;
        public float z;
        public double o;

        public Coordinate(){
        }
    }

    static public class Point{
        public double latitude;
        public double longitude;
        public float altitude;
        public float yaw;
    }
}

