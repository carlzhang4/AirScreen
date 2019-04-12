package com.dji.simulatorDemo;

import android.Manifest;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Handler;
import android.os.Looper;
import android.os.Build;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.os.Bundle;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import dji.common.error.DJISDKError;
import dji.common.flightcontroller.Attitude;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.simulator.InitializationData;
import dji.common.flightcontroller.simulator.SimulatorState;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.model.LocationCoordinate2D;
import dji.common.useraccount.UserAccountState;
import dji.common.util.CommonCallbacks;
import dji.log.DJILog;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import dji.common.error.DJIError;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.sdk.useraccount.UserAccountManager;

public class MainActivity extends Activity implements View.OnClickListener,SensorEventListener {

    private static final String TAG = "cjdebug";
    static public Boolean Enable_recording = new Boolean(false);



    //The SDK Variables
    private FlightControllerState mState;
    private LocationCoordinate3D mLocation;
    private Attitude mAttitude;
    private FlightController mFlightController;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;

    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;



    //UI Variables
    protected TextView mConnectStatusTextView;
    private TextView mTextView;
    private TextView mCoordinateCurrent;
    private TextView mSensorAngle;
    private TextView mCoordinateGoal;



    EditText eX;
    EditText eY;
    EditText eZ;
    EditText eYaw;
    EditText ePid_p;
    EditText ePid_i;
    EditText ePid_d;

    //Personal
    final int PERIOD = 100;
    private Timer mSendVirtualStickDataTimer;
    public Boolean Enable_Sensor_Flag = false;
    public Boolean Enable_Locating_Mode_Flag = false;
    public Pid pidController;

    private float value_x;
    private float value_y;
    private float value_z;
    private float value_yaw;

    private double value_pid_p;
    private double value_pid_i;
    private double value_pid_d;

    double Radius = 1.5;
    double delta_yaw = 0.0;


    //sensor part to get current three angles of phone
    private SensorManager manager;
    private Sensor magneticSensor,accelerometerSensor;
    private float [] values,r,gravity,geomagnetic;
    private double pitch_phone,roll_phone,azimuth_phone;

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()){
            case Sensor.TYPE_MAGNETIC_FIELD:{
                geomagnetic = event.values;
                break;
            }
            case Sensor.TYPE_ACCELEROMETER:{
                gravity = event.values;
                getValue();
                break;
            }
            default:
                break;
        }
    }
    public void getValue(){
        SensorManager.getRotationMatrix(r, null, gravity, geomagnetic);
        SensorManager.getOrientation(r, values);
        azimuth_phone = Math.toDegrees(values[0]);
        pitch_phone = Math.toDegrees(values[1]);
        roll_phone = Math.toDegrees(values[2]);
        mSensorAngle.setText("Azimuth：" + (int)azimuth_phone + "\nPitch：" + (int)pitch_phone + "\nRoll：" + (int)roll_phone);
        if(Enable_Sensor_Flag == true){
            CoordinateConversion.update_goal_coordinate(azimuth_phone+delta_yaw,pitch_phone,Radius);
            updateGoalLoactionUI();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        checkAndRequestPermissions();
        setContentView(R.layout.activity_main);
        initUI();
        //init sensor part
        manager = (SensorManager)getSystemService(SENSOR_SERVICE);
        magneticSensor = manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        accelerometerSensor = manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        values = new float[3];
        gravity = new float[3];
        r = new float[9];
        geomagnetic = new float[3];
        manager.registerListener(this,magneticSensor,SensorManager.SENSOR_DELAY_FASTEST);
        manager.registerListener(this,accelerometerSensor,SensorManager.SENSOR_DELAY_FASTEST);

        // Register the broadcast receiver for receiving the device connection's changes.
        IntentFilter filter = new IntentFilter();
        filter.addAction(DJISimulatorApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);
        Tool.deletePath();

    }






    private void initUI() {

        eX = (EditText)findViewById(R.id.edit_x);
        eY = (EditText)findViewById(R.id.edit_y);
        eZ = (EditText)findViewById(R.id.edit_z);
        eYaw = (EditText)findViewById(R.id.edit_yaw);

        ePid_p = (EditText)findViewById(R.id.edit_pid_p);
        ePid_i = (EditText)findViewById(R.id.edit_pid_i);
        ePid_d = (EditText)findViewById(R.id.edit_pid_d);

        mSensorAngle = (TextView)findViewById(R.id.textview_sensor_angle);
        mConnectStatusTextView = (TextView) findViewById(R.id.ConnectStatusTextView);
        mCoordinateCurrent = (TextView)findViewById(R.id.textview_coordinate_current);
        mCoordinateGoal = (TextView)findViewById(R.id.textview_coordinate_goal);


        Button mStart = (Button)findViewById(R.id.btn_start);
        Button mStop = (Button)findViewById(R.id.btn_stop);
        Button mCalibrate = (Button)findViewById(R.id.btn_calibrate_yaw);
        Button mEnableSensor = (Button)findViewById(R.id.btn_enable_sensor_mode);
        Button mInitBase = (Button)findViewById(R.id.btn_init_base);
        Button mMeassure = (Button)findViewById(R.id.btn_meassure);
        Button mEnableVirtualStick = (Button) findViewById(R.id.btn_enable_virtual_stick);
        Button mDisableVirtualStick = (Button) findViewById(R.id.btn_disable_virtual_stick);
        Button mTakeOff = (Button) findViewById(R.id.btn_take_off);
        Button mLand = (Button) findViewById(R.id.btn_land);


        mStart.setOnClickListener(this);
        mStop.setOnClickListener(this);
        mMeassure.setOnClickListener(this);
        mEnableVirtualStick.setOnClickListener(this);
        mDisableVirtualStick.setOnClickListener(this);
        mTakeOff.setOnClickListener(this);
        mLand.setOnClickListener(this);
        mInitBase.setOnClickListener(this);
        mEnableSensor.setOnClickListener(this);
        mCalibrate.setOnClickListener(this);
    }

    @Override
    public void onClick(View v) {

        switch (v.getId()) {

            case R.id.btn_enable_sensor_mode:{
                Enable_Sensor_Flag = true;
                showToast("Enable Sensor Mode Success!");
                break;
            }
            case R.id.btn_calibrate_yaw:{
                mAttitude = mState.getAttitude();
                delta_yaw = mAttitude.yaw - azimuth_phone;
                showToast("Calibrate Done!");
                break;
            }


            case R.id.btn_init_base:{
                mLocation = mState.getAircraftLocation();
                CoordinateConversion.setBasePoint(mLocation.getLongitude(),mLocation.getLatitude(),mLocation.getAltitude());
                showToast("Init Success!");
                break;
            }

            case R.id.btn_meassure:{
                updateCurrentLoaction();
                updateCurrentLoactionUI();
                break;
            }


            case R.id.btn_enable_virtual_stick:
                if (mFlightController != null){

                    mFlightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null){
                                showToast(djiError.getDescription());
                            }else
                            {
                                showToast("Enable Virtual Stick Success");
                            }
                        }
                    });

                }
                break;

            case R.id.btn_disable_virtual_stick:

                if (mFlightController != null){
                    mFlightController.setVirtualStickModeEnabled(false, new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            if (djiError != null) {
                                showToast(djiError.getDescription());
                            } else {
                                showToast("Disable Virtual Stick Success");
                            }
                        }
                    });
                }
                break;

            case R.id.btn_take_off:
                if (mFlightController != null){
                    mFlightController.startTakeoff(
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if (djiError != null) {
                                        showToast(djiError.getDescription());
                                    } else {
                                        showToast("Take off Success");
                                    }
                                }
                            }
                    );
                }

                break;

            case R.id.btn_land:
                if (mFlightController != null){

                    mFlightController.startLanding(
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
                                    if (djiError != null) {
                                        showToast(djiError.getDescription());
                                    } else {
                                        showToast("Start Landing");
                                    }
                                }
                            }
                    );

                }
                break;

            case R.id.btn_start:{
                Enable_recording = true;
                try{
                    value_x = Float.valueOf(eX.getText().toString());
                    value_y = Float.valueOf(eY.getText().toString());
                    value_z = Float.valueOf(eZ.getText().toString());
                    value_yaw = Float.valueOf(eYaw.getText().toString());
                    value_pid_p = Float.valueOf(ePid_p.getText().toString());
                    value_pid_i = Float.valueOf(ePid_i.getText().toString());
                    value_pid_d = Float.valueOf(ePid_d.getText().toString());
                }
                catch (Exception e){
                    e.printStackTrace();
                    showToast("Invalid value");
                    break;
                }

                pidController = new Pid(value_pid_p,value_pid_i,value_pid_d);

                Enable_Locating_Mode_Flag = true;
                CoordinateConversion.set_goal_coordinate(value_x,value_y,value_z,value_yaw);
                updateGoalLoactionUI();
                showToast("start success!");
                if (null == mSendVirtualStickDataTimer) {
                    mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                    mSendVirtualStickDataTimer = new Timer();
                    mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, PERIOD);
                }
                break;
            }
            case R.id.btn_stop:{

                Enable_recording = false;
                Enable_Sensor_Flag = false;
                Enable_Locating_Mode_Flag = false;
                mYaw = 0;
                mThrottle = 0;
                mPitch = 0;
                mRoll = 0;
                if (null == mSendVirtualStickDataTimer) {
                    mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                    mSendVirtualStickDataTimer = new Timer();
                    mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, PERIOD);
                }
                break;
            }


            default:
                break;
        }
    }

    class SendVirtualStickDataTask extends TimerTask {

        @Override
        public void run() {

            if(Enable_Locating_Mode_Flag == true){
                updateCurrentLoaction();
                updateCurrentLoactionUI();
                Pid.FlyingData returnValue = pidController.calculate(CoordinateConversion.current_coordinate,CoordinateConversion.goal_coordinate);
                Tool.save_type_one(returnValue);
                mYaw = Math.min(returnValue.mYaw,Pid.yawJoyControlMaxSpeed);
                mThrottle = Math.min(returnValue.mThrottle,Pid.verticalJoyControlMaxSpeed);
                mPitch = Math.min(returnValue.mPitch,Pid.pitchJoyControlMaxSpeed);
                mRoll = Math.min(returnValue.mRoll,Pid.rollJoyControlMaxSpeed);
            }


            if (mFlightController != null) {
                mFlightController.sendVirtualStickFlightControlData(
                        new FlightControlData(
                                mPitch, mRoll, mYaw, mThrottle
                        ), new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {

                            }
                        }
                );
            }
            else
                Log.d(TAG, "mFlightcontroller is null!");
        }
    }

    void updateCurrentLoaction(){
        mLocation = mState.getAircraftLocation();
        mAttitude = mState.getAttitude();
        CoordinateConversion.update_current_coordinate(mLocation.getLongitude(),mLocation.getLatitude(),mLocation.getAltitude(),mAttitude.yaw);
    }
    void updateCurrentLoactionUI(){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mCoordinateCurrent.setText("x：" + (double)Math.round(100*CoordinateConversion.current_coordinate.x)/100 + "\ny：" + (double)Math.round(100*CoordinateConversion.current_coordinate.y)/100 + "\nz：" + (double)Math.round(100*CoordinateConversion.current_coordinate.z)/100 + "\nyaw:"+CoordinateConversion.current_coordinate.o);
            }
        });
    }
    void updateGoalLoactionUI(){
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mCoordinateGoal.setText("x：" + (double)Math.round(100*CoordinateConversion.goal_coordinate.x)/100 + "\ny：" + (double)Math.round(100*CoordinateConversion.goal_coordinate.y)/100 + "\nz：" + (double)Math.round(100*CoordinateConversion.goal_coordinate.z)/100 + "\nyaw:"+CoordinateConversion.goal_coordinate.o);
            }
        });

    }






    //Stable parts dont need to change at all***************************************************************************************************
    //Stable parts dont need to change at all***************************************************************************************************
    //Stable parts dont need to change at all***************************************************************************************************
    //Stable parts dont need to change at all***************************************************************************************************
    //Stable parts dont need to change at all***************************************************************************************************

    private void checkAndRequestPermissions() {
        // Check for permissions
        for (String eachPermission : REQUIRED_PERMISSION_LIST) {
            if (ContextCompat.checkSelfPermission(this, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (!missingPermission.isEmpty() && Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this,
                    missingPermission.toArray(new String[missingPermission.size()]),
                    REQUEST_PERMISSION_CODE);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED) {
                    missingPermission.remove(permissions[i]);
                }
            }
        }
        // If there is enough permission, we will start the registration
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else {
            showToast("Missing permissions!!!");
        }
    }

    private void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    showToast( "registering, pls wait...");
                    DJISDKManager.getInstance().registerApp(getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                        @Override
                        public void onRegister(DJIError djiError) {
                            if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                DJILog.e("App registration", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                DJISDKManager.getInstance().startConnectionToProduct();
                                showToast("Register Success");
                            } else {
                                showToast( "Register sdk fails, check network is available");
                            }
                            Log.v(TAG, djiError.getDescription());
                        }

                        @Override
                        public void onProductDisconnect() {
                            Log.d(TAG, "onProductDisconnect");
                            showToast("Product Disconnected");

                        }
                        @Override
                        public void onProductConnect(BaseProduct baseProduct) {
                            Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                            showToast("Product Connected");

                        }
                        @Override
                        public void onComponentChange(BaseProduct.ComponentKey componentKey, BaseComponent oldComponent,
                                                      BaseComponent newComponent) {
                            if (newComponent != null) {
                                newComponent.setComponentListener(new BaseComponent.ComponentListener() {

                                    @Override
                                    public void onConnectivityChange(boolean isConnected) {
                                        Log.d(TAG, "onComponentConnectivityChanged: " + isConnected);
                                    }
                                });
                            }
                            Log.d(TAG,
                                    String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                            componentKey,
                                            oldComponent,
                                            newComponent));
                        }
                    });
                }
            });
        }
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            updateTitleBar();
        }
    };

    public void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }

    private void updateTitleBar() {
        if(mConnectStatusTextView == null) return;
        boolean ret = false;
        BaseProduct product = DJISimulatorApplication.getProductInstance();

        if (product != null) {
            if(product.isConnected()) {
                //The product is connected
                mConnectStatusTextView.setText(DJISimulatorApplication.getProductInstance().getModel() + " Connected");
                ret = true;
            } else {
                if(product instanceof Aircraft) {
                    Aircraft aircraft = (Aircraft)product;
                    if(aircraft.getRemoteController() != null && aircraft.getRemoteController().isConnected()) {
                        // The product is not connected, but the remote controller is connected
                        mConnectStatusTextView.setText("only RC Connected");
                        ret = true;
                    }
                }
            }
        }

        if(!ret) {
            // The product or the remote controller are not connected.
            mConnectStatusTextView.setText("Disconnected");
        }
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        updateTitleBar();
        initFlightController();
        loginAccount();

    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        unregisterReceiver(mReceiver);
        if (null != mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask.cancel();
            mSendVirtualStickDataTask = null;
            mSendVirtualStickDataTimer.cancel();
            mSendVirtualStickDataTimer.purge();
            mSendVirtualStickDataTimer = null;
        }
        super.onDestroy();
    }

    private void loginAccount(){
        UserAccountManager.getInstance().logIntoDJIUserAccount(this,
                new CommonCallbacks.CompletionCallbackWith<UserAccountState>() {
                    @Override
                    public void onSuccess(final UserAccountState userAccountState) {
                        Log.e(TAG, "Login Success");
                    }
                    @Override
                    public void onFailure(DJIError error) {
                        showToast("Login Error:"
                                + error.getDescription());
                    }
                });
    }

    private void initFlightController() {
        Aircraft aircraft = DJISimulatorApplication.getAircraftInstance();
        if (aircraft == null || !aircraft.isConnected()) {
            showToast("Disconnected");
            mFlightController = null;
            return;
        } else {
            mFlightController = aircraft.getFlightController();
            mState = mFlightController.getState();
            mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            mFlightController.getSimulator().setStateCallback(new SimulatorState.Callback() {
                @Override
                public void onUpdate(final SimulatorState stateData) {
                    new Handler(Looper.getMainLooper()).post(new Runnable() {
                        @Override
                        public void run() {

                            String yaw = String.format("%.2f", stateData.getYaw());
                            String pitch = String.format("%.2f", stateData.getPitch());
                            String roll = String.format("%.2f", stateData.getRoll());
                            String positionX = String.format("%.2f", stateData.getPositionX());
                            String positionY = String.format("%.2f", stateData.getPositionY());
                            String positionZ = String.format("%.2f", stateData.getPositionZ());

                        }
                    });
                }
            });
        }
    }

    private static final String[] REQUIRED_PERMISSION_LIST = new String[]{
            Manifest.permission.VIBRATE,
            Manifest.permission.INTERNET,
            Manifest.permission.ACCESS_WIFI_STATE,
            Manifest.permission.WAKE_LOCK,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_NETWORK_STATE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.CHANGE_WIFI_STATE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.READ_PHONE_STATE,
    };
    private List<String> missingPermission = new ArrayList<>();
    private AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private static final int REQUEST_PERMISSION_CODE = 12345;

}
