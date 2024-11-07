package jp.jaxa.iss.kibo.rpc.thailand;

import android.graphics.RectF;

public class BoundingBoxWithScore {
    private RectF rect;
    private float score;
    private String category;

    public BoundingBoxWithScore(RectF rect, float score, String category) {
        this.rect = rect;
        this.score = score;
        this.category = category;
    }

    public RectF getRect() {
        return rect;
    }

    public float getScore() {
        return score;
    }

    public void setScore(float score) {
        this.score = score;
    }

    public String getCategory() {
        return category;
    }
}