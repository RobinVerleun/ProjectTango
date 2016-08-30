package com.example.salim.objectfollower;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

public class GameOverActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_game_over);

        Intent prevIntent = getIntent();
        String score = prevIntent.getStringExtra(ObjectFollowerActivity.MESSAGE);
        TextView endScoreView = (TextView) findViewById(R.id.endScoreView);
        endScoreView.setText("Your time is " + score);
    }

    public void restartGame(View view){
        Intent intent = new Intent(this, ObjectInputActivity.class);
        startActivity(intent);
        finish();
    }
}
