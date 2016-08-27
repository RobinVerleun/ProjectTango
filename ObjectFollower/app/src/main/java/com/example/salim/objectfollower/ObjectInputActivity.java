package com.example.salim.objectfollower;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;

public class ObjectInputActivity extends AppCompatActivity {
    public static final String MESSAGE = "com.example.salim.objectfollower.MESSAGE";
    private TextView initObjectView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_object_input);
        initObjectView = (TextView) findViewById(R.id.initText);
    }

    public void enterGame(){
        Intent intent = new Intent(this, ObjectFollowerActivity.class);
        String numberOfObjects = initObjectView.getText().toString();
        intent.putExtra(MESSAGE, numberOfObjects);
        startActivity(intent);
    }
}
