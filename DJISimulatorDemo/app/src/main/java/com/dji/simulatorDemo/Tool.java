package com.dji.simulatorDemo;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.Calendar;

public class Tool {
    static public void save_type_one(Pid.FlyingData returnValue){
        StringBuilder m = new StringBuilder();
        m.append(keep(returnValue.mPitch)+" ");
        m.append(keep(returnValue.mRoll)+" ");
        m.append(keep(returnValue.mThrottle)+" ");
        m.append(keep(returnValue.mYaw)+" ");



        m.append(keep(CoordinateConversion.current_coordinate.x)+" ");
        m.append(keep(CoordinateConversion.current_coordinate.y)+" ");
        m.append(keep(CoordinateConversion.current_coordinate.z)+" ");
        m.append(keep(CoordinateConversion.current_coordinate.o)+" ");

        m.append(keep(CoordinateConversion.goal_coordinate.x)+" ");
        m.append(keep(CoordinateConversion.goal_coordinate.y)+" ");
        m.append(keep(CoordinateConversion.goal_coordinate.z)+" ");
        m.append(keep(CoordinateConversion.goal_coordinate.o)+" ");
        m.append("\n");

        try{
            save(m.toString(),"flyingdata");
        }
        catch (Exception e){

        }
    }

    static public void save_type_two(double longitude, double latitude, float altitude, double yaw){
        if(MainActivity.Enable_recording == false)
            return;
        StringBuilder m = new StringBuilder();
        m.append(longitude+" ");
        m.append(latitude+" ");
        m.append(altitude+" ");
        m.append(yaw+" ");


        m.append(CoordinateConversion.basePoint.longitude+" ");
        m.append(CoordinateConversion.basePoint.latitude+" ");
        m.append(CoordinateConversion.basePoint.altitude+" ");
        m.append(CoordinateConversion.basePoint.yaw+" ");

        m.append(keep(CoordinateConversion.current_coordinate.x)+" ");
        m.append(keep(CoordinateConversion.current_coordinate.y)+" ");
        m.append(keep(CoordinateConversion.current_coordinate.z)+" ");
        m.append(keep(CoordinateConversion.current_coordinate.o)+" ");
        m.append("\n");

        try{
            save(m.toString(),"convertingData");
        }
        catch (Exception e){

        }
    }

    static public void save_PID(double p,double i,double d) throws IOException{

        String pid = new String(p+"_"+i+"_"+d);
        File sdCard = Environment.getExternalStorageDirectory();
        // 获取的sd卡的绝对路径为 /storage/sdcard
        sdCard = new File(sdCard, "/AAirScreen");
        sdCard.mkdirs();// 创建MyFiles目录
        sdCard = new File(sdCard, "pid.txt");
        FileOutputStream out = new FileOutputStream(sdCard);
        Writer writer = new OutputStreamWriter(out);
        try {
            writer.write(pid);
        } finally {
            writer.close();
        }
    }

    static public void save(String content, String fileName) throws IOException {
        if(MainActivity.Enable_recording == false)
            return;
        Calendar calendar = Calendar.getInstance();
        int hour = calendar.get(Calendar.HOUR_OF_DAY);
        int minute = calendar.get(Calendar.MINUTE);
        int second = calendar.get(Calendar.SECOND);
        long mSecond = calendar.getTimeInMillis();

        String time = new String(hour+":"+minute+":"+second+" "+mSecond+" ");
        File sdCard = Environment.getExternalStorageDirectory();
        // 获取的sd卡的绝对路径为 /storage/sdcard
        sdCard = new File(sdCard, "/AAirScreen");
        sdCard.mkdirs();// 创建MyFiles目录
        sdCard = new File(sdCard, fileName+".txt");
        FileOutputStream out = new FileOutputStream(sdCard,true);
        Writer writer = new OutputStreamWriter(out);
        try {
            writer.write(time+content);
        } finally {
            writer.close();
        }
    }

    public static void deletePath() {
        File file;
        file = Environment.getExternalStorageDirectory();
        file = new File(file.getPath() + "/AAirScreen");
        RecursionDeleteFile(file);

    }
    public static void RecursionDeleteFile(File file) {
        if (file.isFile()) {
            file.delete();
            return;
        }
        if (file.isDirectory()) {
            File[] childFile = file.listFiles();
            if (childFile == null || childFile.length == 0) {
                file.delete();
                return;
            }
            for (File f : childFile) {
                RecursionDeleteFile(f);
            }
            file.delete();
        }
    }


    static public String keep(double x){
        return String.format("%.2f",x);
    }

    static public String keep(float x){
        return String.format("%.2f",x);
    }

}
