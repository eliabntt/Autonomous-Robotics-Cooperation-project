# Homework 1 - Perception

## Principal aspects

- *Rviz* configuration for a better visualization of the work environment
- *Listener* for [apriltags](http://wiki.ros.org/apriltags_ros) on topic `/tag_detections` to obtain at the same time, for each object in the Kinect FOV, tag's:
    - ID
    - Size
    - Pose with respect to `camera_rgb_optical_frame`
- Transformation of the pose's coordinates to have them with respect to the `world` reference frame
- Creation of two *publisher* that expose, w.r.t. `world` frame:
    1. the requested tags that have to be grabbed with the future modules
    2. the not requested tags which will become collision objects to avoid 
- Output of these informations in `output.txt` (inside module folder) as requested in the assignment.

## Commands (for simulation)

In separated terminals launc:

- challenge_arena in Gazebo:  
    ```
    roslaunch challenge_arena challenge.launch sim:=true
    ```
- apriltags:  
    ```
    roslaunch challenge_arena apriltag.launch
    ```
- Rviz to see the results *(optional)*:  
    ```
    rosrun rviz rviz -d `rospack find g01_perception`/rviz/our_config.rviz
    ```
- Perception module:  
    ```
    roslaunch g01_perception discover.launch 
        [ids:="[[frame_id],]"] [sim:=true/false] [forever:=true/false]
    ```
    
    - `ids` is meant to keep the list of the wanted tags (separated by `,`) that we want to detect:
        if found, they will be saved in the output file and published within the related topic.
        The eventually remaining tags found will be published within the other topic as explained above.
        If the parameter is not present or is empty all tags will be flagged as wanted.
        Each invalid tag will cause an error message.
    - `sim` parameter is used to distinguish between simulated and real environment, mainly for offsett related values.
    - `forever` is used to keep the perception module running, continuosly publishing its topics.

## Additional notes
    
- Default values:  
    
    - ```sim:=true```  
    - ```forever:=false```  
    - ```ids:=""```  
    
- If invalid values will be given to `sim` or `forever`, default behaviour is used.
