package it.cnr.istc.sirobotics;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import pl.droidsonroids.gif.GifImageView;

public class QuestionActivity extends AppCompatActivity {

    private static final String TAG = "SI-Robotics";
    private GifImageView image_view;
    private TextView question_text;
    private LinearLayout answers_layout;
    private final LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().requestFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_question);
        image_view = findViewById(R.id.image_view);
        question_text = findViewById(R.id.question_text);
        answers_layout = findViewById(R.id.answers_layout);

        Intent intent = getIntent();
        String message = intent.getStringExtra(MainActivity.QUESTION);
        try {
            JSONObject msg_obj = new JSONObject(message);

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

            question_text.setText(msg_obj.getString("text"));

            JSONArray buttons = msg_obj.getJSONArray("buttons");
            Log.i(TAG, "Adding " + buttons.length() + " buttons..");
            for (int i = 0; i < buttons.length(); i++) {
                final JSONObject c_btn = buttons.getJSONObject(i);
                Button btn = new Button(this);
                btn.setText(c_btn.getString("text"));
                btn.setOnClickListener(view -> {
                    final JSONObject ans = new JSONObject();
                    try {
                        ans.put("type", "answer_question");
                        ans.put("intent", c_btn.getString("intent"));
                    } catch (JSONException e) {
                        Log.e(TAG, "", e);
                    }
                    WSConnection.getInstance().send(ans.toString());
                    finish();
                });
                Log.i(TAG, "Adding " + i + "th button..");
                answers_layout.addView(btn, lp);
            }
        } catch (JSONException e) {
            Log.e(TAG, "Cannot parse message..", e);
        }
    }
}