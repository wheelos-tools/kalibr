remote: car-ningde-orin
path: /home/nvidia/02code/kalibr
set up the camera2camera extrinsic calibration yamls for four camera pairs 68->66->64->65->67 .camera only has 30% duplication area with neighbor camera.
Calibration board AprilGrid 6x6 tags, size=8.8cm and spacing=2.64cm
cameras:

- { uri: "rtsp://admin:Nvidia135@192.168.1.68:554/live", codec: "h265", width: 1920, height: 1080, fps: 25 }
- { uri: "rtsp://admin:Nvidia135@192.168.1.66:554/live", codec: "h265", width: 1920, height: 1080, fps: 25 }
- { uri: "rtsp://admin:Nvidia135@192.168.1.64:554/live", codec: "h265", width: 1920, height: 1080, fps: 25 }
- { uri: "rtsp://admin:Nvidia135@192.168.1.65:554/live", codec: "h265", width: 1920, height: 1080, fps: 25 }
- { uri: "rtsp://admin:Nvidia135@192.168.1.67:554/live", codec: "h265", width: 1920, height: 1080, fps: 25 }
