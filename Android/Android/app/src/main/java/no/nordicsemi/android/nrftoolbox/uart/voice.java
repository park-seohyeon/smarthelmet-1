package no.nordicsemi.android.nrftoolbox.uart;
import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Intent;
import android.os.Build;
import android.os.StrictMode;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.sql.*;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;


import no.nordicsemi.android.nrftoolbox.R;

public class voice extends AppCompatActivity{
    Intent intent;
    SpeechRecognizer mRecognizer;
    Button sttBtn;
    TextView textView;
    final int PERMISSION = 1;

    TextView textViewA;
    Connection conn = null;
    final String TAG = "TEST";
    int numA = 1;

    String str = null;



    protected void onCreate(Bundle savedInstanceState){
    super.onCreate(savedInstanceState);
    setContentView(R.layout.voice_home);
    //textViewA = (TextView)findViewById(R.id.textView2);


    if ( Build.VERSION.SDK_INT >= 23 ){
            // 퍼미션 체크
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.INTERNET,
                    Manifest.permission.RECORD_AUDIO},PERMISSION);
        }

        textView = (TextView)findViewById(R.id.sttResult);
        sttBtn = (Button) findViewById(R.id.sttStart);

        intent=new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE,getPackageName());
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE,"ko-KR");
        sttBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mRecognizer = SpeechRecognizer.createSpeechRecognizer(voice.this);
                mRecognizer.setRecognitionListener(listener);
                mRecognizer.startListening(intent);

            }

        });
    }

        private RecognitionListener listener = new RecognitionListener() {
            @Override
            public void onReadyForSpeech(Bundle params) {
                Toast.makeText(getApplicationContext(),"음성인식을 시작합니다.",Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onBeginningOfSpeech() {}

            @Override
            public void onRmsChanged(float rmsdB) {}

            @Override
            public void onBufferReceived(byte[] buffer) {}

            @Override
            public void onEndOfSpeech() {}

            @Override
            public void onError(int error) {
                String message;

                switch (error) {
                    case SpeechRecognizer.ERROR_AUDIO:
                        message = "오디오 에러";
                        break;
                    case SpeechRecognizer.ERROR_CLIENT:
                        message = "클라이언트 에러";
                        break;
                    case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                        message = "퍼미션 없음";
                        break;
                    case SpeechRecognizer.ERROR_NETWORK:
                        message = "네트워크 에러";
                        break;
                    case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                        message = "네트웍 타임아웃";
                        break;
                    case SpeechRecognizer.ERROR_NO_MATCH:
                        message = "찾을 수 없음";
                        break;
                    case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                        message = "RECOGNIZER가 바쁨";
                        break;
                    case SpeechRecognizer.ERROR_SERVER:
                        message = "서버가 이상함";
                        break;
                    case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                        message = "말하는 시간초과";
                        break;
                    default:
                        message = "알 수 없는 오류임";
                        break;
                }

                Toast.makeText(getApplicationContext(), "에러가 발생하였습니다. : " + message,Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onResults(Bundle results) {

                ArrayList<String> matches =
                        results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);

                for(int i = 0; i < matches.size() ; i++){
                    textView.setText(matches.get(i));
                    str = matches.get(i);
                }
            }

            @Override
            public void onPartialResults(Bundle partialResults) {}

            @Override
            public void onEvent(int eventType, Bundle params) {}
        };



    @SuppressLint("NewApi")
    public void testFunctionWithPOST(View v) throws SQLException, ClassNotFoundException, InstantiationException, IllegalAccessException {
        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        Class.forName("net.sourceforge.jtds.jdbc.Driver");
        String connectURL = "jdbc:jtds:sqlserver://hwayun.database.windows.net:1433;DatabaseName=fourdong;user=hwayun@hwayun;password=ghkdbs0!;encrypt=true;trustServerCertificate=false;hostNameInCertificate=*.database.windows.net;loginTimeout=30;";
        conn = DriverManager.getConnection(connectURL);
        Timer timer = new Timer();
        TimerTask TT = new TimerTask() {
        //Thread threadA = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    if(str == null) {
                        try {
                            // 1. 현재 DB가 한글 인식 못함 -> 수정하거나 새로 만드세요
                            // 2. 1번 하고 이거 실행했을 때 한글 안 넘어가면
                            //    -> 쿼리문을 utf-8로 인코딩 후 전송
                            if(UARTManager.sss[0]==(byte)0xF1){
                                int[] sensdata =  new int[UARTManager.sss.length];

                                for(int i = 0; i < sensdata.length; i++){
                                    sensdata[i] = (UARTManager.sss[i] & (byte)0xff);
                                }

                                String query = "insert into dbo.temperature values (" + sensdata[1] + ", CURRENT_TIMESTAMP" + ")";
                                Log.e(TAG, query);
                                Statement stmt = conn.createStatement();
                                stmt.executeQuery(query);
                                Log.e(TAG, "Insert success!");
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        textViewA.setText("온도");
                                    }
                                });
                            }else if(UARTManager.sss[0]==(byte)0xF2){
                                int[] sensdata =  new int[UARTManager.sss.length];

                                for(int i = 0; i < sensdata.length; i++){
                                    sensdata[i] = (UARTManager.sss[i] & (byte)0xff);
                                }

                                String query = "insert into dbo.humidity values (" + sensdata[1] + ", CURRENT_TIMESTAMP" + ")";
                                Log.e(TAG, query);
                                Statement stmt = conn.createStatement();
                                stmt.executeQuery(query);
                                Log.e(TAG, "Insert success!");
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        textViewA.setText("습도");
                                    }
                                });
                            }else if(UARTManager.sss[0]==(byte)0xF3){
                                int[] sensdata =  new int[UARTManager.sss.length];

                                for(int i = 0; i < sensdata.length; i++){
                                        sensdata[i] = (UARTManager.sss[i] & (byte)0xff);
                                }
                                String query = "insert into dbo.gas values (" + sensdata[1] + "," + sensdata[2] + ", CURRENT_TIMESTAMP" + ")";
                                Log.e(TAG, query);
                                Statement stmt = conn.createStatement();
                                stmt.executeQuery(query);
                                Log.e(TAG, "Insert success!");
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        textViewA.setText("가스");
                                    }
                                });
                            }else if(UARTManager.sss[0]==(byte)0xF4){
                                int[] sensdata =  new int[UARTManager.sss.length];

                                for(int i = 0; i < sensdata.length; i++){
                                    if(i>=1) {
                                        if(i==2) break;
                                        sensdata[i] = ((UARTManager.sss[i] & (byte)0xff) << 8) | (UARTManager.sss[i+1] & (byte)0xff);
                                    }
                                    else{
                                        sensdata[i] = (UARTManager.sss[i] & (byte)0xff);
                                    }
                                }

                                String query = "insert into dbo.accel values (" + sensdata[1] + ", CURRENT_TIMESTAMP" + ")";
                                Log.e(TAG, query);
                                Statement stmt = conn.createStatement();
                                stmt.executeQuery(query);
                                Log.e(TAG, "Insert success!");
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        textViewA.setText("가속도");
                                    }
                                });
                            }else if(UARTManager.sss[0]==(byte)0xF5){
                                int[] sensdata =  new int[UARTManager.sss.length];

                                for(int i = 0; i < sensdata.length; i++){
                                    if(i>=1) {
                                        if(i==2 || i==4 || i==6) break;
                                        sensdata[i] = ((UARTManager.sss[i] & (byte)0xff) << 8) | (UARTManager.sss[i+1] & (byte)0xff);
                                    }
                                    else{
                                        sensdata[i] = (UARTManager.sss[i] & (byte)0xff);
                                    }
                                }

                                String query = "insert into dbo.gravity values (" + sensdata[1] + "," + sensdata[2] + "," + sensdata[3] + ", CURRENT_TIMESTAMP" + ")";
                                Log.e(TAG, query);
                                Statement stmt = conn.createStatement();
                                stmt.executeQuery(query);
                                Log.e(TAG, "Insert success!");
                                runOnUiThread(new Runnable() {
                                    @Override
                                    public void run() {
                                        textViewA.setText("중력");
                                    }
                                });
                            }else{
                                return;
                            }

                            //String query = "insert into dbo.noridic values ('" + Build.ID + "','"+ sensdata[0] + "'," + sensdata[1] + ")";
                            //  stmt.close();
                        } catch(Exception ex) {
                            Log.d(TAG, "Exception: "+ex);
                            ex.printStackTrace();
                        }
                    }

                    // 1. 현재 DB가 한글 인식 못함 -> 수정하거나 새로 만드세요
                    // 2. 1번 하고 이거 실행했을 때 한글 안 넘어가면
                    //    -> 쿼리문을 utf-8로 인코딩 후 전송

                  //  stmt.close();
                } catch(Exception ex) {
                    Log.d(TAG, "Exception: "+ex);
                    ex.printStackTrace();
                }
            }
        };
        timer.schedule(TT, 0, 1500); //Timer 실행

        //threadA.start();
        if(str!=null) {
            Thread threadA = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        String query = "insert into dbo.voice values ('" + str + "', CURRENT_TIMESTAMP" + ")";
                        Log.e(TAG, query);
                        Statement stmt = conn.createStatement();
                        stmt.executeQuery(query);
                        Log.e(TAG, "Insert success!");
                    } catch (Exception ex) {
                        Log.d(TAG, "Exception: " + ex);
                        ex.printStackTrace();
                    }
                }
            });
            threadA.start();
        }
    }



    //메인화면에서 사원버튼누르면 azure sql databases로 데이터전송
    @SuppressLint("NewApi")
    public void inputsend(String vava) throws SQLException, ClassNotFoundException, InstantiationException, IllegalAccessException{
        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        Class.forName("net.sourceforge.jtds.jdbc.Driver");
        String connectURL = "jdbc:jtds:sqlserver://hwayun.database.windows.net:1433;DatabaseName=fourdong;user=hwayun@hwayun;password=ghkdbs0!;encrypt=true;trustServerCertificate=false;hostNameInCertificate=*.database.windows.net;loginTimeout=30;";
        conn = DriverManager.getConnection(connectURL);

        Thread threadA = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    if(vava.substring(5).equals("출근")){
                        String query = "insert into dbo.ImHereIN values ('" + vava.substring(0,4) + "', CURRENT_TIMESTAMP" + ")";
                        Log.e(TAG, query);
                        Statement stmt = conn.createStatement();
                        stmt.executeQuery(query);
                        Log.e(TAG, "Insert success!");
                    }
                    else if(vava.substring(5).equals("퇴근")){
                        String query = "insert into dbo.ImHereOUT values ('" + vava.substring(0,4) + "', CURRENT_TIMESTAMP" + ")";
                        Log.e(TAG, query);
                        Statement stmt = conn.createStatement();
                        stmt.executeQuery(query);
                        Log.e(TAG, "Insert success!");
                    }
                    else if(vava.substring(5).equals("야근")){
                        String query = "insert into dbo.ImHereNIGHT values ('" + vava.substring(0,4) + "', CURRENT_TIMESTAMP" + ")";
                        Log.e(TAG, query);
                        Statement stmt = conn.createStatement();
                        stmt.executeQuery(query);
                        Log.e(TAG, "Insert success!");
                    }
                } catch (Exception ex) {
                    Log.d(TAG, "Exception: "+ex);
                    ex.printStackTrace();
                }
            }
        });
        threadA.start();
    }
}

