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

    private FlightControllerState mState;
    private LocationCoordinate3D mLocation;
    private Attitude mAttitude;

    private FlightController mFlightController;
    protected TextView mConnectStatusTextView;
    private Button mBtnEnableVirtualStick;
    private Button mBtnDisableVirtualStick;
    private Button mBtnTakeOff;
    private Button mBtnLand;
    private Button mMeassure;
    private Button mInitBase;
    private Button mEnableSensor;
    private Button mCalibrate;

    private TextView mTextView;
    private TextView mCoordinate;
    private TextView textview_sensor_angle;
    private TextView mCoordinate_Goal;

    private CoordinateConversion mConverter;


    private Timer mSendVirtualStickDataTimer;
    private SendVirtualStickDataTask mSendVirtualStickDataTask;

    public Boolean Enable_Sensor_Flag = false;

    private float mPitch;
    private float mRoll;
    private float mYaw;
    private float mThrottle;

    private float rotate_value;
    private float vertical_value;
    private float forword_value;
    private float direction_value;
    private float time_value = 0;

    private int period_value = 100; //ms

    private double pid_p_value;
    private double pid_i_value;
    private double pid_d_value;

    EditText rotate;
    EditText vertical;
    EditText forward;
    EditText direction;
    EditText time;
    EditText period;

    EditText pid_p;
    EditText pid_i;
    EditText pid_d;

    float verticalJoyControlMaxSpeed = 2;
    float yawJoyControlMaxSpeed = 30;
    float pitchJoyControlMaxSpeed = 10;
    float rollJoyControlMaxSpeed = 10;

    double Radius = 3.0;
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
        textview_sensor_angle.setText("Azimuth：" + (int)azimuth_phone + "\nPitch：" + (int)pitch_phone + "\nRoll：" + (int)roll_phone);
        if(Enable_Sensor_Flag == true){
            mLocation = mState.getAircraftLocation();
            mAttitude = mState.getAttitude();
            CoordinateConversion.Coordinate currentCoordinate = mConverter.convert(mLocation.getLongitude(),mLocation.getLatitude(),mLocation.getAltitude());
            CoordinateConversion.Coordinate goalCoordinate = mConverter. new Coordinate(azimuth_phone+delta_yaw,pitch_phone,Radius);
            mCoordinate_Goal.setText("x：" + (double)Math.round(100*goalCoordinate.x)/100 + "\ny：" + (double)Math.round(100*goalCoordinate.y)/100 + "\nz：" + (double)Math.round(100*goalCoordinate.z)/100);

            double delta_x = goalCoordinate.x - currentCoordinate.x;
            double delta_y = goalCoordinate.y - currentCoordinate.y;
            double delta_z = goalCoordinate.z - currentCoordinate.z;
            double delta_o = mAttitude.yaw;
            int x_flag;
            int y_flag;
            int z_flag;
            int o_flag;
            if(delta_x>0.3)
                x_flag = 1;
            else if(delta_x<-0.3)
                x_flag = -1;
            else x_flag = 0;

            if(delta_y>0.3)
                y_flag = 1;
            else if(delta_y<-0.3)
                y_flag = -1;
            else y_flag = 0;

            if(delta_z>0.1)
                z_flag = 1;
            else if(delta_z<-0.1)
                z_flag = -1;
            else z_flag = 0;

            if(delta_o>5)
                o_flag = 1;
            else if(delta_o<-5)
                o_flag = -1;
            else o_flag = 0;

            mYaw = (float)(yawJoyControlMaxSpeed * o_flag * -1.0 * 0.2);
            mThrottle = (float)(verticalJoyControlMaxSpeed * z_flag * 0.05);
            mPitch = (float)(pitchJoyControlMaxSpeed * x_flag * 0.05);
            mRoll = (float)(rollJoyControlMaxSpeed * y_flag * 0.05);

            if (null == mSendVirtualStickDataTimer) {
                mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                mSendVirtualStickDataTimer = new Timer();
                mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, period_value);

            }


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
    }


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

                            mTextView.setText("Yaw : " + yaw + ", Pitch : " + pitch + ", Roll : " + roll + "\n" + ", PosX : " + positionX +
                                    ", PosY : " + positionY +
                                    ", PosZ : " + positionZ);
                        }
                    });
                }
            });
        }
    }

    private void initUI() {

        rotate = (EditText)findViewById(R.id.rotate);
        vertical = (EditText)findViewById(R.id.vertical);
        forward = (EditText)findViewById(R.id.forword);
        direction = (EditText)findViewById(R.id.direction);
        time = (EditText)findViewById(R.id.time);
        period = (EditText)findViewById(R.id.period);

        pid_p = (EditText)findViewById(R.id.pid_p);
        pid_i = (EditText)findViewById(R.id.pid_i);
        pid_d = (EditText)findViewById(R.id.pid_d);

        textview_sensor_angle = (TextView)findViewById(R.id.textview_sensor_angle);

        Button start = (Button)findViewById(R.id.start);
        Button stop = (Button)findViewById(R.id.stop);


        start.setOnClickListener(this);
        stop.setOnClickListener(this);

        mCalibrate = (Button)findViewById(R.id.calibrate_yaw);
        mEnableSensor = (Button)findViewById(R.id.btn_enable_sensor_mode);
        mInitBase = (Button)findViewById(R.id.btn_initBase);
        mMeassure = (Button)findViewById(R.id.btn_meassure);
        mBtnEnableVirtualStick = (Button) findViewById(R.id.btn_enable_virtual_stick);
        mBtnDisableVirtualStick = (Button) findViewById(R.id.btn_disable_virtual_stick);
        mBtnTakeOff = (Button) findViewById(R.id.btn_take_off);
        mBtnLand = (Button) findViewById(R.id.btn_land);
        mTextView = (TextView) findViewById(R.id.textview_simulator);
        mConnectStatusTextView = (TextView) findViewById(R.id.ConnectStatusTextView);
        mCoordinate = (TextView)findViewById(R.id.textview_coordinate);
        mCoordinate_Goal = (TextView)findViewById(R.id.textview_coordinate_goal);

        mMeassure.setOnClickListener(this);
        mBtnEnableVirtualStick.setOnClickListener(this);
        mBtnDisableVirtualStick.setOnClickListener(this);
        mBtnTakeOff.setOnClickListener(this);
        mBtnLand.setOnClickListener(this);
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
            case R.id.calibrate_yaw:{
                mAttitude = mState.getAttitude();
                delta_yaw = mAttitude.yaw - azimuth_phone;
                showToast("Calibrate Done!");
                break;
            }


            case R.id.btn_initBase:{
                mLocation = mState.getAircraftLocation();
                mAttitude = mState.getAttitude();
                mConverter = new CoordinateConversion(mLocation.getLongitude(),mLocation.getLatitude(),mLocation.getAltitude());
                showToast("Init Success!");
                break;
            }

            case R.id.btn_meassure:{
                mLocation = mState.getAircraftLocation();
                mAttitude = mState.getAttitude();
                Log.d(TAG, "La"+String.valueOf(mLocation.getLatitude()));
                Log.d(TAG, "Lo"+String.valueOf(mLocation.getLongitude()));
                CoordinateConversion.Coordinate currentCoordinate = mConverter.convert(mLocation.getLongitude(),mLocation.getLatitude(),mLocation.getAltitude());
                mCoordinate.setText("x：" + (double)Math.round(100*currentCoordinate.x)/100 + "\ny：" + (double)Math.round(100*currentCoordinate.y)/100 + "\nz：" + (double)Math.round(100*currentCoordinate.z)/100 + "\nyaw:"+mAttitude.yaw);

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
            case R.id.start:{


                try{
                    rotate_value = Float.valueOf(rotate.getText().toString());
                    vertical_value = Float.valueOf(vertical.getText().toString());
                    forword_value = Float.valueOf(forward.getText().toString());
                    direction_value = Float.valueOf(direction.getText().toString());
                    //time_value = 1000*Float.valueOf(time.getText().toString());
                    period_value = Integer.valueOf(period.getText().toString());

                    if(Math.abs(rotate_value)>1 || Math.abs(vertical_value)>1 || Math.abs(forword_value)>1 || Math.abs(direction_value)>1 || period_value<=10){
                        period_value = 100;
                        showToast("value must set between -1 to 1 and period must bigger than 10");
                        break;
                    }
                    else {
                        mYaw = (float)(yawJoyControlMaxSpeed * rotate_value * -1.0);
                        mThrottle = (float)(verticalJoyControlMaxSpeed * vertical_value);
                        mPitch = (float)(pitchJoyControlMaxSpeed * direction_value * -1.0);
                        mRoll = (float)(rollJoyControlMaxSpeed * forword_value);
                        showToast(String.valueOf(rotate_value) + "/" + String.valueOf(vertical_value) + "/" + String.valueOf(forword_value) + "/" + String.valueOf(direction_value) + "/" + String.valueOf(time_value) + "/" + String.valueOf(period_value));
                    }
                }
                catch (Exception e){
                    e.printStackTrace();
                    showToast("Invalid value");
                    break;
                }
                if (null == mSendVirtualStickDataTimer) {
                    mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                    mSendVirtualStickDataTimer = new Timer();
                    mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, period_value);

                }
                break;
            }
            case R.id.stop:{

                mYaw = 0;
                mThrottle = 0;
                mPitch = 0;
                mRoll = 0;
                period_value = 100;
                Enable_Sensor_Flag = false;

                if (null == mSendVirtualStickDataTimer) {
                    mSendVirtualStickDataTask = new SendVirtualStickDataTask();
                    mSendVirtualStickDataTimer = new Timer();
                    mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, period_value);
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

            mLocation = mState.getAircraftLocation();
            mAttitude = mState.getAttitude();

            Log.d(TAG, "height:"+mLocation.getAltitude());
            Log.d(TAG, "Latitude:"+mLocation.getLatitude());
            Log.d(TAG, "Longtitude:"+mLocation.getLongitude());
            Log.d(TAG, "Yaw:"+mAttitude.yaw);

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

}
