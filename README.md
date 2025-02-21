# Robot_project

### Current issues regarding mapping

- The map file is being created successfully, but the detected objects are inconsistently placed. Objects often appear in incorrect locations, and the detection system frequently misidentifies items—such as bags or hands—or merges multiple objects into one. This results in an inaccurate representation of object positions on the map.
- Once object detection is improved, generating an accurate map should be straightforward using the current implementation.

# Status:
Wait for detection. 

# Current work:
Occumṕancy grid node.

# To-DO week 9
- Make new rosbags with slower movement when rotating.
  -- I believe that the lidar scans can be improved with slower rotational speed when the robot is turning. So probably when implementing the movements for the robot we will want to generally use slwoer speeds so that the lidar can keep up.
- Make rosbag for a simulated workspace such as the one with the tape on the floor and add boxes and likewise as simulated obstacles.
  -- Purpose for this is to see how much noise there generally will be in order to figure out ways to filter these out in the end. 
