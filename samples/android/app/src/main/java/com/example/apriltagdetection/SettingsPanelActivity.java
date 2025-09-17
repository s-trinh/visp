package com.example.apriltagdetection;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import androidx.appcompat.app.AppCompatActivity;

public class SettingsPanelActivity extends AppCompatActivity {
    private static final String TAG = "SettingsPanelActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        EditText cameraFocalInput = findViewById(R.id.cameraFocalInput);
        EditText tagSizeInput = findViewById(R.id.tagSizeInput);

        Button validate = findViewById(R.id.btnValidateSettings);
        validate.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.d(TAG, "SettingsPanelActivity::validate::onClick()");

                String cameraFocalInput_str = cameraFocalInput.getText().toString();
                String tagSizeInput_str = tagSizeInput.getText().toString();

                Intent resultIntent = new Intent();
                resultIntent.putExtra("cameraFocalValue", cameraFocalInput_str);
                resultIntent.putExtra("tagSizeValue", tagSizeInput_str);
                setResult(RESULT_OK, resultIntent);


                finish();
            }
        });
    }
}
