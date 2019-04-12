package com.dji.simulatorDemo;

public class Pid {
    double delta_x = 0;
    double delta_y = 0;
    double delta_z = 0;
    double delta_o = 0;

    double delta_x_1 = 0;
    double delta_y_1 = 0;
    double delta_z_1 = 0;
    double delta_o_1 = 0;

    double delta_x_2 = 0;
    double delta_y_2 = 0;
    double delta_z_2 = 0;
    double delta_o_2 = 0;


    double pid_p,pid_i,pid_d;

    static final float verticalJoyControlMaxSpeed = 2;
    static final float yawJoyControlMaxSpeed = 30;
    static final float pitchJoyControlMaxSpeed = 10;
    static final float rollJoyControlMaxSpeed = 10;

    public FlyingData returnData;

    public Pid(double pid_p,double pid_i,double pid_d){
        this.pid_p = pid_p;
        this.pid_i = pid_i;
        this.pid_d = pid_d;
        returnData = new FlyingData();

    }
    FlyingData calculate(CoordinateConversion.Coordinate location_current, CoordinateConversion.Coordinate location_goal){

        delta_x_2 = delta_x_1;
        delta_y_2 = delta_y_1;
        delta_z_2 = delta_z_1;
        delta_o_2 = delta_o_1;

        delta_x_1 = delta_x;
        delta_y_1 = delta_y;
        delta_z_1 = delta_z;
        delta_o_1 = delta_o;

        delta_x = location_goal.x - location_current.x;
        delta_y = location_goal.y - location_current.y;
        delta_z = location_goal.z - location_current.z;
        delta_o = location_goal.o - location_current.o;

        double pidP = 0.14;
        double pidI = 0.016;
        double pidD = 0.1;
        try {
            Tool.save_PID(pidP, pidI, pidD);
        }catch (Exception e){

        }
        returnData.mYaw += (5 * ((delta_o-delta_o_1) + 0*delta_o + 0*(delta_o - 2*delta_o_1 +delta_o_2)))*(yawJoyControlMaxSpeed)/180;
        returnData.mThrottle += (0.3 * ((delta_z-delta_z_1) + 0*delta_z + 0*(delta_z - 2*delta_z_1 +delta_z_2)))*(verticalJoyControlMaxSpeed);
        returnData.mRoll += (pidP * ((delta_y-delta_y_1) + pidI*delta_y + pidD*(delta_y - 2*delta_y_1 +delta_y_2)))*(rollJoyControlMaxSpeed);
        returnData.mPitch += (pidP* ((delta_x-delta_x_1) + pidI*delta_x + pidD*(delta_x - 2*delta_x_1 +delta_x_2)))*(pitchJoyControlMaxSpeed);
        return returnData;
    }


    class FlyingData{
        public float mPitch;
        public float mRoll;
        public float mYaw;
        public float mThrottle;
        public FlyingData(){
            mPitch = 0;
            mRoll = 0;
            mYaw = 0;
            mThrottle = 0;
        }
    }
}
