# Welcome to Astronut Team code for 5th Kibo Robot Programming Challenge
## This code we were using for KIBORPC5 with 1'st score award for Thailand Preliminary round and for Final Round with 1'st score 
<img src="/Readme Images/Awards.jpg">

## This code has been tested about 1000~ Run according to Officials
## This code is the same code that we use in 5th-KIBO RPC final round. As you see in our [app folder](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand) <br />
Our code contains a large number of classes. We will explain how our code works later! <br />
List of classes <br />
-[YourService](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/YourService.java) <br />
-[NonMaxSuppression](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/NonMaxSuppression.java) [**More Info**](https://github.com/KIBO-Astronut/5th-KIBO/blob/main/explain_class/Readme_Text/NMS.md) <br />
-[BoundingBoxWithScore](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/BoundingBoxWithScore.java) <br />
-[ARResult](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/ARResult.java) <br />
-[Quaternion](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/Quaternion.java) <br />
-[QuaternionUtils](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/QuaternionUtils.java) 

## For Archived Machine Learning Model please go to [Model folder](https://github.com/KIBO-Astronut/5th-KIBO/tree/main/Tensorflow%20Lite%20model) 
## Here are some Finals Round Images from ISS
## For more images please go to [Images folder](https://github.com/KIBO-Astronut/5th-KIBO/tree/main/Readme%20Images/ImageFromISS)

# Image distorted in Astrobee's camera due to fish eye len

<img src="Readme Images/ImageFromISS/EMR-2.png"> 

### Distorted image
<img src="Readme Images/ImageFromISS/Pre-2.png"> 

### Undistorted image via Calib3d.undistort() by opencv
# Target 1
<img src="Readme Images/ImageFromISS/Pre-1.png">

### Inital Target 1 Images

<img src="Readme Images/ImageFromISS/post_1.png">

### After images Processing by AR tracking

<img src="Readme Images/ImageFromISS/before_iou_1.png">

### Running Model

<img src="Readme Images/ImageFromISS/after_iou_1.png">

### Final result target 1

# Target 2

<img src="Readme Images/ImageFromISS/Pre-2.png">

### Inital Target 2 Images

<img src="Readme Images/ImageFromISS/post_2.png">

### After images Processing by AR tracking

<img src="Readme Images/ImageFromISS/before_iou_2.png">

### Running Model

<img src="Readme Images/ImageFromISS/after_iou_2.png">

### Final result target 2
# Target 3

<img src="Readme Images/ImageFromISS/Pre-3.png">

### Inital Target 3 Images

<img src="Readme Images/ImageFromISS/post_3.png">

### After images Processing by AR tracking

<img src="Readme Images/ImageFromISS/before_iou_3.png">

### Running Model
#### We can see it got some hallucination

<img src="Readme Images/ImageFromISS/after_iou_3.png">

### After filter the result it gone
If you have any questions, leave in discussions are available.



