package jp.jaxa.iss.kibo.rpc.thailand;

import android.graphics.RectF;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

public class NonMaxSuppression {
    private static boolean isNearEdge(RectF rect, float imageWidth, float imageHeight, float margin_in_per) {
        float margin_width = (margin_in_per/100)*imageWidth;
        float margin_height = (margin_in_per/100)*imageHeight;
        return rect.left < margin_width || rect.top < margin_height || rect.right > imageWidth - margin_width || rect.bottom > imageHeight - margin_height;
    }

    public static List<BoundingBoxWithScore> softNonMaxSuppression(List<BoundingBoxWithScore> boxes, float sigma, float Nt, float threshold, float imageWidth, float imageHeight, float margin, int method) {
        List<BoundingBoxWithScore> D = new ArrayList<>();
        for (BoundingBoxWithScore box : boxes) {
            if (isNearEdge(box.getRect(), imageWidth, imageHeight, margin)) {
                if(box.getScore() > 0.5) {
                    D.add(box);
                }
            }
            else {
                D.add(box);
            }
        }

        int N = D.size();

        for (int i = 0; i < N; i++) {
            // Find the box with the maximum score
            int maxpos = i;
            float maxscore = D.get(i).getScore();
            for (int j = i + 1; j < N; j++) {
                if (D.get(j).getScore() > maxscore) {
                    maxscore = D.get(j).getScore();
                    maxpos = j;
                }
            }

            // Swap
            BoundingBoxWithScore temp = D.get(i);
            D.set(i, D.get(maxpos));
            D.set(maxpos, temp);

            // IoU calculation
            RectF maxBox = D.get(i).getRect();
            for (int j = i + 1; j < N; j++) {
                RectF box = D.get(j).getRect();
                float iou = iou(maxBox, box);

                // Apply Soft-NMS
                float weight;
                if (method == 1) {  // linear
                    weight = iou > Nt ? 1 - iou : 1;
                } else if (method == 2) {  // gaussian
                    weight = (float) Math.exp(-(iou * iou) / sigma);
                } else {  // original NMS
                    weight = iou > Nt ? 0 : 1;
                }

                D.get(j).setScore(D.get(j).getScore() * weight);
            }
        }

        // Filter out boxes with score below threshold
        List<BoundingBoxWithScore> result = new ArrayList<>();
        for (BoundingBoxWithScore box : D) {
            if (box.getScore() > threshold) {
                Log.i("Score","confident_score after NMS :"+box.getScore());
                result.add(box);
            }
        }

        return result;
    }


    private static float iou(RectF a, RectF b) {
        float intersectionLeft = Math.max(a.left, b.left);
        float intersectionTop = Math.max(a.top, b.top);
        float intersectionRight = Math.min(a.right, b.right);
        float intersectionBottom = Math.min(a.bottom, b.bottom);

        float intersectionWidth = Math.max(0, intersectionRight - intersectionLeft);
        float intersectionHeight = Math.max(0, intersectionBottom - intersectionTop);

        float intersectionArea = intersectionWidth * intersectionHeight;
        float areaA = a.width() * a.height();
        float areaB = b.width() * b.height();
        float unionArea = areaA + areaB - intersectionArea;
        Log.i("IOU", "IOU_threshold: " + (intersectionArea / unionArea));

        return intersectionArea / unionArea;
    }
}