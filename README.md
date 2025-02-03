# Welcome to Astronut Team code for 5th Kibo Robot Programming Challenge
## This code we were using for KIBORPC5 with 1'st score award for Thailand Preliminary round and for Final Round with 1'st score 
<img src="/Readme Images/Awards.jpg">


## This code has been tested about 1000~ Run according to Officials
## Check out code feedback from JAXA team <a href="#feedback"> here! </a>

## This code is the same code that we use in 5th-KIBO RPC final round. As you see in our [app folder](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand) <br />
### Our code contains a large number of classes. We will explain how our code works later! 
### List of classes 
- [YourService](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/YourService.java) 
- [NonMaxSuppression](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/NonMaxSuppression.java)
  - [More Info](https://github.com/KIBO-Astronut/5th-KIBO/blob/main/explain_class/Readme_Text/NMS.md)
- [BoundingBoxWithScore](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/BoundingBoxWithScore.java) 
- [ARResult](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/ARResult.java) 
- [Quaternion](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/Quaternion.java) 
- [QuaternionUtils](app/src/main/java/jp/jaxa/iss/kibo/rpc/thailand/QuaternionUtils.java) 

## For Archived Machine Learning Model please go to [Model folder](https://github.com/KIBO-Astronut/5th-KIBO/tree/main/Tensorflow%20Lite%20model) 
## Here are some Finals Round Images from ISS
## For more images please go to [Images folder](https://github.com/KIBO-Astronut/5th-KIBO/tree/main/Readme%20Images/ImageFromISS)

---
# Example of Data

## Image distorted in Astrobee's camera due to fish eye len
### Distorted image

<img src="Readme Images/ImageFromISS/EMR-2.png"> 

### Undistorted image via Calib3d.undistort() by opencv


<img src="Readme Images/ImageFromISS/Pre-2.png"> 



# Target 1
### Inital Target 1 Images

<img src="Readme Images/ImageFromISS/Pre-1.png">

### After images Processing by AR tracking

<img src="Readme Images/ImageFromISS/post_1.png">

### Running Model

<img src="Readme Images/ImageFromISS/before_iou_1.png">

### Final result target 1

<img src="Readme Images/ImageFromISS/after_iou_1.png">


# Target 2
### Inital Target 2 Images

<img src="Readme Images/ImageFromISS/Pre-2.png">

### After images Processing by AR tracking

<img src="Readme Images/ImageFromISS/post_2.png">

### Running Model

<img src="Readme Images/ImageFromISS/before_iou_2.png">

### Final result target 2

<img src="Readme Images/ImageFromISS/after_iou_2.png">

# Target 3
### Inital Target 3 Images

<img src="Readme Images/ImageFromISS/Pre-3.png">

### After images Processing by AR tracking

<img src="Readme Images/ImageFromISS/post_3.png">

### Running Model
#### We can see it got some hallucination
<img src="Readme Images/ImageFromISS/before_iou_3.png">


### After filter the result it gone

<img src="Readme Images/ImageFromISS/after_iou_3.png">

If you have any questions, leave in discussions are available.

# Feedback
## Code Feedback from JAXA
<img src="Readme Images/thailand_Feedback/Slide1.png">
<img src="Readme Images/thailand_Feedback/Slide2.png">
<img src="Readme Images/thailand_Feedback/Slide3.png">
<img src="Readme Images/thailand_Feedback/Slide4.png">



