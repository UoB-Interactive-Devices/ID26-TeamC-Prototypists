# ID26-TeamC-Prototypists

## TAs ##
- Primary TA: Wenda Zhao
- Assistant TA: Yuan Liang

<h2>Robotic Arm -></h2>

**Ideas**
- Tidying rooms -> Most likely
- Kitchen helper, cooking, passing objects and ingredients
- Toilet cleaner, to grab objects and use them to clean the toilets

**TODO**
- Print out robot arm components
- Perhaps buy chasis to put robot arm on for movement around room?


**Links**
- none yet


**Overleaf link**
- Overleaf Project: https://www.overleaf.com/project/69931118b01cc7a0acb87528


**Abstract**
Robotic arms could help with everyday tasks at home, but current gesture control systems mostly rely on short and simple demonstrations in controlled settings. Performance drops when motions are long, tasks involve many steps, or environments change over time.
In this project we do not use an IMU glove. Instead, we use a camera based vision system to learn robot arm control from human demonstration. User hand and arm motions are captured visually. OpenCV is used for video preprocessing and coordinate extraction. Roboflow is used to manage and augment datasets. A YOLO based model detects hands, objects, and interaction states in real time. As you perform natural actions, the system aligns visual signals with robot actions and learns a policy that can replay and adapt the behavior safely. The goal is reliable reproduction and adaptation in real environments.
We evaluate the approach on tasks related to tidying a kid’s room. Tasks include picking up toys, sorting items into bins, placing books on shelves, and arranging objects on a desk. We report success rate, task completion time, and generalization across users.
