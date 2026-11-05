
#!/bin/bash


while true; do
  

  rostopic pub -1 /left_hand_control sensor_msgs/JointState "{
    header: {
      seq: 0,
      stamp: {secs: 0, nsecs: 0},
      frame_id: ''
    },
    name: ['joint3','joint4'],
    position: [255, 255, 0, 0, 0, 0, 128, 128, 128, 128, 255, 0, 0, 0, 0, 0],
    velocity: [0],
    effort: [0]
  }"


  
done
