# In NonMaxSuppression class you will dicover two main function isNearEdge and softNonMaxSuppression
## First we will talk about isNearEdge
### isNearEdge is function that check if the result bounding box near on the edge, it deletes the result near the edge.
### So why? Why we decided to do hillarious things like that? When we use AR to cropping image we decided cropping image bigger than the actual one <br />
### so the hallucination result would likely to appear on the edge of the image
<img src="/explain_class/Readme_Assets/testing1.jpg">

## Next, softNonMaxSuppression
### Soft Non Max Suppression also known as Soft-NMS, It is method used to filter halucinate result and the next level of Non Max Suppression.
### Soft-NMS is method sorting the result based on their score and intersection over union:IOU, It will decay score of hallucinate result to be lower than our theshold

Let deep dive in Soft-NMS method <br />
Soft-NMS is filter hallucinate result that overlap the actual result like the following image:
<img src="/explain_class/Readme_Assets/demos_image.JPG">
We have 3 method that you can choose <br />
*Linear decay
*Gaussian decay
*Regular NMS

### Linear Soft-NMS
If IOU value is more than IOU theshold we will decay score of item by using following equation
<img src="/explain_class/Readme_Assets/equ1.JPG">
<img src="/explain_class/Readme_Assets/equ2.JPG"> <br />
If New score is less than theshold, the result may be hallucinate.

### Gaussian Soft-NMS
If two bounding box overlap each other, Gaussian Soft-NMS will decay score of the item by using gaussian decay <br />
Note: This method no need IOU theshold.
<img src="/explain_class/Readme_Assets/equ3.JPG">
If New score is less than theshold, the result may be hallucinate.


### Regular NMS
If two bounding box have IOU value more than IOU theshold, It will be delete. <br />

So we decided to choose gaussian due to it flexible method. <br />
If you want more infomation, let's leave it in the discussion.


