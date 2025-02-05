author: Frank Tränkle[^1]  
Hochschule Heilbronn, Germany
bibliography: ../lib/bib.bib
csl: ../lib/ieee.csl
link-citations: true
reference-section-title: References
title: MAD76 Vision

Markers
=======

MAD76 applies AuUco markers in computer vision for detecting and
tracking cars. This section explains

-   how to generate and print the markers (see
    Section <a href="#marker-generation" data-reference-type="ref" data-reference="marker-generation">1.1</a>),

-   how to place the coordinate frame markers (see
    Section <a href="#frame-markers" data-reference-type="ref" data-reference="frame-markers">1.2</a>),

-   how to place the car markers (see
    Section <a href="#car-markers" data-reference-type="ref" data-reference="car-markers">1.3</a>),

-   and how to calibrate the Raspberry Pi camera using ChArUco boards
    (see
    Section <a href="#camera-calibration" data-reference-type="ref" data-reference="camera-calibration">1.4</a>).

Marker Generation
-----------------

<figure>
<img src="markers.png" id="F-markers" alt="" /><figcaption>AruCo markers for cars and coordinate frame.</figcaption>
</figure>

-   The cars are tracked by ArUco markers \[[1](#ref-opencv-aruco)\].

-   Computer vision computes the Cartesian coordinates and the yaw
    angles of the cars.

-   The markers are generated with the OpenCV ArUco library.

-   A custom ArUco dictionary of 8 markers with a size of 3x3 bits is
    used to increase the reliability of computer vision.

-   The PNG image of the 8 markers can be optionally created by

        cd ~/src/mad2/mad_ws
        install/mbmadvisionaruco/lib/mbmadvisionaruco/create_board --bb=1 -d=17 -w=4 -h=2 -l=200 -s=100 markers.png

-   The markers IDs are from 0 to 7, 0 to 3 in the first row from left
    to right, and 4 to 7 in the second row.

-   Print the markers in
    Fig. <a href="#F-markers" data-reference-type="ref" data-reference="F-markers">1</a>
    on a snow-white, 80 grams paper.

    -   Make sure to configure high quality printing.

    -   Scale the printing such that the black area of the markers have
        a height and width of 21mm each.

-   Cut the markers as squares including approx. 5mm boundaries.

-   Note the marker IDs before cutting with a thin pencil on the
    boundaries, because you will need these IDs later on.

Frame Markers
-------------

<figure>
<img src="track.png" id="F-framemarkers" alt="" /><figcaption>Track with 4 coordinate frame markers.</figcaption>
</figure>

-   4 frame markers define the coordinate frame of the track.

-   All coordinates of cars and track are measured in meters.

-   The frame origin $(s_{01},s_{02})=(0\mathrm{m},0\mathrm{m})$ is at
    the center point of marker ID4.

-   Place frame markers with IDs 4, 5, 6, 7 at corners of board as
    depicted in figure.

    -   It is recommended to apply the distances as depicted.

    -   Although modified distances may be later configured in the ROS2
        package `mbmadvisionaruco`.

    -   The distances are measured at the marker center points.

    -   The markers must form a rectangle.

    -   The sequence of the marker IDs is essential.

Car Markers
-----------

<figure>
<img src="carmarker.jpg" id="F-carmarker" alt="" /><figcaption>Red car with marker ID 0.</figcaption>
</figure>

-   Each car has its individual marker.

-   The following configuration is recommended:

    | Marker ID | Car          |
    |:----------|:-------------|
    | 0         | red orange   |
    | 1         | green yellow |
    | 2         | blue         |
    | 3         | white        |

-   If you have fewer than four cars, please start with ID 0 in any
    case.

-   Each marker’s center point must be placed exactly at the car’s rear
    axle center point.

-   The horizontal orientation of the marker must match to the forward
    direction of the car.

Camera Calibration
------------------

<figure>
<img src="charucoboard.png" id="F-charuco" alt="" /><figcaption>ChArUco board for camera calibration.</figcaption>
</figure>

The Raspberry Pi camera must be calibrated, so that the MAD76 can
undistort the camera image frames \[[2](#ref-ros-camera-calibration)\].
The calibration is performed applying an ChArUco board, which is an
augmentation of a chess board by AruCo markers for higher precision.
Follow the following steps for calibrating your camera:

-   Print the marker board in
    Fig. <a href="#F-charuco" data-reference-type="ref" data-reference="F-charuco">4</a>
    on a snow-white DIN-A4 paper. Use high-quality printer settings.

-   This PNG image can optionally be created by

        cd ~/src/mad2/mad_ws
        install/mbmadvisionaruco/lib/mbmadvisionaruco/create_board_charuco -d=0 -w=7 -h=5 -ml=500 -sl=800 charucoboard.png

-   Fix this paper on a cardboard.

-   Measure the side lengths of the squares and the AruCo markers (not
    including the white boundaries) in meters.

-   Calibrate the camera by running the following command:

        ros2 run camera_calibration cameracalibrator --pattern=charuco --size 7x5 --square 0.036 --charuco_marker_size 0.022 --aruco_dict 4x4_50 image:=/mad/camera/image_raw camera:=/mad/camera camera/set_camera_info:=/mad/camera/set_camera_info

-   After successful calibration the camera matrix and distortion
    coefficients are stored in the file

        ~/.ros/camera_info/imx296__base_axi_pcie_120000_rp1_i2c_88000_imx296_1a_800x600.yaml

    or similar.

-   This calibration data file will then be automatically loaded by the
    MAD76 computer vision for undistorting camera frames.

References [bibliography]
==========

<div id="refs" class="references" markdown="1">

<div id="ref-opencv-aruco" markdown="1">

\[1\] OpenCV, “Detection of ArUco Markers.” 2025. Available:
<https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>

</div>

<div id="ref-ros-camera-calibration" markdown="1">

\[2\] ROS, “How to Calibrate a Monocular Camera.” 2025. Available:
<https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration>

</div>

</div>

[^1]: frank.traenkle@hs-heilbronn.de
