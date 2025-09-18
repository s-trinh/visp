package com.example.apriltagdetection;

import android.content.Intent;
import android.os.Bundle;
import android.text.method.LinkMovementMethod;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

public class SettingsPanelActivity extends AppCompatActivity {
    private static final String TAG = "SettingsPanelActivity";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        TextView originalCreditDesc = findViewById(R.id.originalCreditDesc);
        originalCreditDesc.setMovementMethod(LinkMovementMethod.getInstance());

        TextView apriltagDesc = findViewById(R.id.apriltagDesc);
        apriltagDesc.setMovementMethod(LinkMovementMethod.getInstance());

        TextView onlineMarkers = findViewById(R.id.onlineMarkers);
        onlineMarkers.setMovementMethod(LinkMovementMethod.getInstance());

        // Update values
        Intent intent = getIntent();
        float camera_focal_mm = intent.getFloatExtra("camera_focal_mm", 0);
        TextView camera_focal_mm_value = findViewById(R.id.camera_lens_focal_mm_value);
        camera_focal_mm_value.setText(camera_focal_mm + " mm");

        int camera_native_resolution_w = intent.getIntExtra("camera_native_w", 0);
        int camera_native_resolution_h = intent.getIntExtra("camera_native_h", 0);
        TextView camera_native_resolution_value = findViewById(R.id.camera_native_resolution_value);
        camera_native_resolution_value.setText(camera_native_resolution_w + " X " + camera_native_resolution_h);

        float camera_hfov = intent.getFloatExtra("camera_hfov", 0);
        float camera_wfov = intent.getFloatExtra("camera_wfov", 0);
        TextView camera_fov_value = findViewById(R.id.camera_fov_value);
        camera_fov_value.setText(camera_hfov + "° (hfov) " + camera_wfov + "° (vfov)");

        double camera_sensor_w = intent.getDoubleExtra("camera_sensor_w", 0);
        double camera_sensor_h = intent.getDoubleExtra("camera_sensor_h", 0);
        TextView camera_sensor_size_value = findViewById(R.id.camera_sensor_size_value);
        camera_sensor_size_value.setText(String.format("%.3f", camera_sensor_w) + " mm X " + String.format("%.3f", camera_sensor_h) + " mm");

        int image_w = intent.getIntExtra("image_w", 0);
        int image_h = intent.getIntExtra("image_h", 0);
        TextView image_resolution_value = findViewById(R.id.image_resolution_value);
        image_resolution_value.setText(image_w + " X " + image_h);

        EditText cameraFocalInput = findViewById(R.id.cameraFocalInput);
        double focalLength = intent.getDoubleExtra("focal", 0);
        cameraFocalInput.setText(String.valueOf(focalLength));

        EditText tagSizeInput = findViewById(R.id.tagSizeInput);
        double tagSize = intent.getDoubleExtra("tag_size", 0);
        tagSizeInput.setText(String.valueOf(tagSize));

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
