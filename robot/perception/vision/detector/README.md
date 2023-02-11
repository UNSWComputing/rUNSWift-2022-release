

# Table of Contents
- [BallDetector](#BallDetector)
- [RegionFieldFeatureDetector](#RegionFieldFeatureDetector)
- [RobotDetector](#RobotDetector)
- [SSRobotDetector](#SSRobotDetector)

# BallDetector <a name="BallDetector"></a>
_To do_

Refer to [ReadTheDocs](https://runswift.readthedocs.io/en/latest/perception/vision/ball_detector.html)

# RegionFieldFeatureDetector <a name="RegionFieldFeatureDetector"></a>
_To do_

Refer to [ReadTheDocs](https://runswift.readthedocs.io/en/latest/perception/vision/field_feature_detector.html)

# RobotDetector <a name="RobotDetector"></a>
The __RobotDetector__ is an older version of the robot detector that is still used on the Nao V5 robots. You can read more about it in the [ReadTheDocs](https://runswift.readthedocs.io/en/latest/perception/vision/robot_detector.html).

# SSRobotDetector <a name="SSRobotDetector"></a>
The __SSRobotDetector__ is the new robot detector that runs on the Nao V6 robots. It implements a convolutional neural network based on [Yolov5](https://github.com/ultralytics/yolov5) with added modifications to ensure it is fast enough for our purposes. It functions by taking in a downsampled grayscaled image from the top camera and passing it through the neural network. The candidate robot bounding boxes are then extracted from the output of the neural network where non-max suppression is then performed to give the final bounding boxes. These final bounding boxes are then stored on the __Blackboard__.

The __SSRobotDetector__ file also contains another experimental implementation which can be switch to by uncommenting the `#define RD_USE_PIXEL_CLASSIFIER` macro. Rather than outputing candidate bounding boxes, the convolutional neural network outputs the probability of a pixel belonging to a robot. It then guesses the robot bounding boxes by taking those regions and drawing bounding boxes around them using connected-component analysis. Currently it is neither as accurate nor performant as the first implementation. 

The weights for the neural network are stored in `ml_models/dnn_models`. The networks can be trained and tested [here](https://drive.google.com/drive/folders/1FnI1d5BYZW2gV7-l7X6gHI5uFQKPmJg3?usp=sharing) using PyTorch.
