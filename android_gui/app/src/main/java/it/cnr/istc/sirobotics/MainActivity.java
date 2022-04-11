package it.cnr.istc.sirobotics;

import android.content.Context;
import android.content.Intent;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONException;
import org.json.JSONObject;

import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;
import pl.droidsonroids.gif.GifImageView;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "SI-Robotics";
    public static final String QUESTION = "it.cnr.istc.sirobotics.QUESTION";
    private GifImageView image_view;
    private Button talk_to_me;
    private final GUIWebSocketListener ws_listener = new GUIWebSocketListener(this);

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().requestFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);

        image_view = findViewById(R.id.image_view);
        talk_to_me = findViewById(R.id.talk_to_me);
        talk_to_me.setOnClickListener(view -> {
            final JSONObject ans = new JSONObject();
            try {
                ans.put("type", "talk_to_me");
            } catch (JSONException e) {
                Log.e(TAG, "", e);
            }
            WSConnection.getInstance().send(ans.toString());
        });

        try {
            ApplicationInfo ai = getPackageManager().getApplicationInfo(getPackageName(), PackageManager.GET_META_DATA);
            Bundle bundle = ai.metaData;
            WSConnection.getInstance().setHost(bundle.getString("gui_host"));
            WSConnection.getInstance().setPort(Integer.toString(bundle.getInt("gui_port")));
        } catch (PackageManager.NameNotFoundException e) {
            Log.e(TAG, "Cannot retrieve application parameters..", e);
        }

        WSConnection.getInstance().setListener(ws_listener);
        WSConnection.getInstance().connect();
    }

    @Override
    protected void onStart() {
        super.onStart();
        image_view.setImageResource(R.drawable.idle);
    }

    private class GUIWebSocketListener extends WebSocketListener {

        private final Context ctx;

        private GUIWebSocketListener(Context ctx) {
            this.ctx = ctx;
        }

        @Override
        public void onMessage(WebSocket webSocket, String text) {
            Log.i(TAG, "Received message: " + text);
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    try {
                        JSONObject msg_obj = new JSONObject(text);
                        switch (msg_obj.getString("type")) {
                            case "dialogue_state":
                                talk_to_me.setEnabled(msg_obj.getInt("dialogue_state") == 0);
                                break;
                            case "show_face":
                                switch (msg_obj.getString("facial_expression")) {
                                    case "happy":
                                        image_view.setImageResource(R.drawable.happy);
                                        break;
                                    case "happy_talking":
                                        image_view.setImageResource(R.drawable.happy_talking);
                                        break;
                                    case "idle":
                                        image_view.setImageResource(R.drawable.idle);
                                        break;
                                    case "laugh":
                                        image_view.setImageResource(R.drawable.laugh);
                                        break;
                                    case "listening":
                                        image_view.setImageResource(R.drawable.listening);
                                        break;
                                    case "sad":
                                        image_view.setImageResource(R.drawable.sad);
                                        break;
                                    case "sad_talking":
                                        image_view.setImageResource(R.drawable.sad_talking);
                                        break;
                                    case "talking":
                                        image_view.setImageResource(R.drawable.talking);
                                        break;
                                }
                                break;
                            case "show_image":
                                switch (msg_obj.getString("src")) {
                                    case "static/images/biceps_curl.gif":
                                        image_view.setImageResource(R.drawable.biceps_curl);
                                        break;
                                }
                                break;
                            case "ask_question": {
                                Intent intent = new Intent(ctx, QuestionActivity.class);
                                intent.putExtra(QUESTION, text);
                                startActivity(intent);
                            }
                            break;
                        }
                    } catch (JSONException e) {
                        Log.e(TAG, "Cannot parse message..", e);
                    }
                }
            });
        }

        @Override
        public void onFailure(WebSocket webSocket, Throwable t, @Nullable Response response) {
            Log.e(TAG, "Connection failure..", t);
            runOnUiThread(new Runnable() {
                public void run() {
                    Toast.makeText(getApplicationContext(), "Connection failure..", Toast.LENGTH_SHORT).show();
                }
            });

            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                Log.e(TAG, "Interrupted thread..", e);
            }
            Log.i(TAG, "Attempting to reestablish connection..");
            WSConnection.getInstance().connect();
        }

        @Override
        public void onOpen(WebSocket webSocket, Response response) {
            Log.i(TAG, "Connection established!");
            runOnUiThread(new Runnable() {
                public void run() {
                    Toast.makeText(getApplicationContext(), "Connection established!", Toast.LENGTH_SHORT).show();
                }
            });
        }

        @Override
        public void onClosed(WebSocket webSocket, int code, String reason) {
            Log.w(TAG, "Connection closed..");
            runOnUiThread(new Runnable() {
                public void run() {
                    Toast.makeText(getApplicationContext(), "Connection closed..", Toast.LENGTH_SHORT).show();
                }
            });

            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                Log.e(TAG, "Interrupted thread..", e);
            }
            Log.i(TAG, "Attempting to reestablish connection..");
            WSConnection.getInstance().connect();
        }
    }
}