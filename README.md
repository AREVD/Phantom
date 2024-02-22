# Camera Sensor-Embedded Phantom Model for External Ventricular Drain
This repository contains the developmental artifacts for the custom phantom skull model used in the paper _"Did I Do Well? Instantaneous Assessment of Trainees' Performance in Augmented Reality-assisted Neurosurgical Training"_, submitted to **the 5th Annual Workshop on 3D Content Creation for Simulated Training in eXtended Reality, co-located with IEEE VR, 2024** by [Sangjun Eom](https://sites.duke.edu/sangjuneom/), [Tiffany Ma](https://sites.duke.edu/tiffanyma/), [Tianyi Hu](http://hutianyi.tech/), Neha Vutakuri, Joshua Jackson, and [Maria Gorlatova](https://maria.gorlatova.com/). The repository includes all of the relevant code and assets necessary for the implementation of real-time distance calculation.

# Outline
1. [Overview](https://github.com/AREVD/Phantom/blob/main/README.md#overview)
2. [Hardware Setup](https://github.com/AREVD/Phantom/blob/main/README.md#hardware-setup)
3. [Software Setup](https://github.com/AREVD/Phantom/blob/main/README.md#hardware-setup)
4. [Quick Start](https://github.com/AREVD/Phantom/blob/main/README.md#quick-start)

# Overview

The integration of Augmented Reality (AR) in neurosurgery provides guidance to surgeons through the visualization of patient-specific anatomy by enhancing their field of view. Our lab has developed an AR-assisted training system for a neurosurgical procedure, the placement of an external ventricular drain (EVD), using marker-based tracking of a phantom model and surgical tool. EVD is performed in order to alleviate pressure built in the ventricular system of the brain due to excess cerebrospinal fluid (CSF). In this procedure, surgeons drill a small hole in the skull through which they place a catheter that drains the CSF to an external closed system.

Our AR system provides real-time feedback to the user, who performs a catheter insertion through a pre-drilled hole in the phantom skull model. The guidance includes a distance-to-target value as an evaluation of the user's accuracy. This value is measured from the tip of the catheter to the location of the target point, the foramen of Monro. The distance is computed in real-time using a stereo-camera setup within the phantom model. The rest of this README details both the hardware and software implementation of this stereo-camera setup.

# Hardware Setup

## Materials

### Raspberry Pi 
1. [Raspberry Pi 4 Model B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
2. (2x) [Arducam Fisheye Camera for Raspberry Pi](https://www.arducam.com/product/arducam-ultra-wide-angle-fisheye-5mp-ov5647-camera-for-raspberry-pi/)
3. [Arducam Multi Camera Adapter for Raspberry Pi](https://www.arducam.com/product/multi-camera-v2-1-adapter-raspberry-pi/)
4. (2x) [A1 FFCs 60cm Raspberry Pi Camera Extension Cable](https://a.co/d/9ydk6Bi)

### Gelatin Mold
1. [SimbaLux Acrylic Sheets](https://a.co/d/h83qMPt)
2. [CHANZON White LED Diode Lights](https://a.co/d/ftPlEu8)
3. [PandaHall Metal Beads](https://a.co/d/ffelK5M)
4. [Custom Collagen SuperClear Gelatin](https://customcollagenshop.com/products/superclear-gelatin)

### Additional Tools and Materials
1. Hot glue and hot glue gun
2. Nuts and bolts for securing cameras onto camera mounts
3. Electrical tape for securing camera mounts to gelatin mold
4. (2x) Female to female DuPont wires (~50cm)
5. Laser cutter or precision knife
6. [CODMAN BACTISEAL EVD Catheter Set](https://www.integralife.com/codman-bactiseal-evd-catheter-set/product/neurocritical-care-evd-catheters-bactiseal-anti-microbial-catheter-codman-bactiseal-evd-catheter-set)
7. Colored polish/paint or silicone dye

All 3D models were printed using PLA filament and Ultimaker 3 printers. Black filament was used to print the gelatin mold and camera mounts to create a high-contrast environment for the cameras, and light grey filament was used to print the phantom skull to present a non-distracting visual experience to the user.

## Camera Hardware Setup

<p align="center">
  <img src="https://github.com/AREVD/Phantom/blob/main/Figures/figure1.jpg" width=50% height=50%>
</p>
<p align="center">
Figure 1. Camera hardware setup steps.
	</p>
 
Required 3D print: [Camera_mounts.stl](https://github.com/AREVD/Phantom/blob/main/3D%20Models/Camera_mounts.stl)

1. Replace cameras' native cables with the 60cm extension cables.
2. Secure cameras to camera mounts using nuts and bolts on the four corners of each camera (Fig. 1a, 1b).
3. Connect cameras to the multi camera adapter's A and C ports using the extension cables (Fig. 1c).
4. Mount the adapter to pins 1-26 of the Raspberry Pi. Secure with nuts and bolts if desired.
5. Connect the adapter's camera cable to the Raspberry Pi.

## Gelatin Mold Setup

<p align="center">
  <img src="https://github.com/AREVD/Phantom/blob/main/Figures/figure2.jpg" width=50% height=50%>
</p>
<p align="center">
Figure 2. Gelatin mold setup steps.
	</p>

Required 3D print: [Gelatin_mold.stl](https://github.com/AREVD/Phantom/tree/main/3D%20Models/Gelatin_mold.stl)

1. Cut acrylic sheets into 2-inch squares. Two squares are needed per mold.
2. Secure squares from the inside of the gelatin mold using hot glue along all edges of the squares (Fig. 2a, 2b).
3. Secure LED diode light to the bottom corner between the two camera windows using hot glue (Fig. 2c).
4. Secure 6mm metal bead to the spherical indentation within the mold (Fig. 2d).
5. Ensure the mold is water-tight by filling it with water; if not, add hot glue as necessary.
6. Connect the LED diode light to pins 32 (GPIO 12) and 34 (Ground) using DuPont wires (Fig. 1c). 

## Gelatin Mixture Instructions

<p align="center">
  <img src="https://github.com/AREVD/Phantom/blob/main/Figures/figure3.jpg" width=50% height=50%>
</p>
<p align="center">
Figure 3. Gelatin mixture setup steps.
	</p>

The following steps yield three molds' worth of gelatin mixture.

1. Weigh out 20g of gelatin and combine with 250ml of cold water. Mix until clumping is no longer visualized (Fig. 3a). Let sit for 10-15 minutes.
2. Heat 700ml of water until steam is visualized. Closely monitor and do not allow to boil. A microwave is sufficient for heating.
3. Combine the gelatin mixture with the larger volume of water. Mix thoroughly until mixture is clear (Fig. 3b).
4. Cool mixture to room temperature. This can be accelerated by refrigerating the mixture. *Note: Closely monitor to prevent excessive cooling, which would cause the gelatin to coagulate prematurely.*
5. Disconnect the gelatin molds' LED diode light from the DuPont wires.
6. Suspend the molds in a cup or other container using paper towels or other supports such that the circular opening faces upwards and is parallel to the ground.
7. Fill the molds with the mixture up to the opening (Fig. 3c).
8. Refrigerate for at least 1 hour.

## Final Hardware Setup

<p align="center">
  <img src="https://github.com/AREVD/Phantom/blob/main/Figures/figure4.jpg" width=50% height=50%>
</p>
<p align="center">
Figure 4. Final hardware setup steps.
	</p>

Required 3D prints: [Top_skull_pegs.stl](https://github.com/AREVD/Phantom/tree/main/3D%20Models/Top_skull_pegs.stl), [Bottom_skull_pegs.stl](https://github.com/AREVD/Phantom/tree/main/3D%20Models/Bottom_skull_pegs.stl)

1. Thread the DuPont wires through the holes in the bottom of the phantom into the inside of the phantom (Fig. 4a).
2. Connect the DuPont wires to the LED diode light of the gelatin mold (Fig. 4b).
3. Secure the camera mounts onto the gelatin mold using electrical tape. Position A at the back of the phantom and C on the right side of the phantom.
4. Insert the mold into the peg in the center of the phantom.
5. Close the phantom by connecting the top of the skull to the bottom of the skull. Secure with tape if necessary.
6. Color the tip of the catheter using polish, paint, or silicone dye in a 2mm region (Fig. 4c). Our development uses a green color, but a different bright color that looks distinct from the rest of the environment (mold color, catheter color) may be used, although some colors may be suboptimal.

# Software Setup

*Note: Our development uses a Raspberry Pi 4 Model B running Raspbian OS. The steps below may not work as expected on different operating systems.

1. Install the following packages using a Raspberry Pi terminal.

```
sudo apt install python3-opencv
sudo apt-get install python3-numpy
sudo apt-get install python3-picamera2
sudo apt-get install python3-scipy
sudo apt-get install python3-matplotlib
pip install keyboard
```

2. Disable the legacy camera option in Raspberry Pi configuration settings. If this option cannot be found in the settings menu, the configuration menu can be accessed by typing `raspi-config` into a terminal. A reboot will be required.

*Note: For troubleshooting regarding the multi camera adapter, please refer to the [Arducam documentation of the product.](https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/Multi-Camera-CamArray/#arducam-multi-camera-adapter-board)*

## Quick Start

<p align="center">
  <img src="https://github.com/AREVD/Phantom/blob/main/Figures/figure5.gif" width=50% height=50%>
</p>
<p align="center">
Figure 5. Quick start application.
	</p>

We provide a straightforward script, [AR.py](https://github.com/AREVD/Phantom/blob/main/Quick%20Start/AR.py), 
for real-time distance calculation, as well as a video feed from both cameras, utilizing camera calibration 
optimized for our experimental setup (Fig. 5). The images and code used for calibration can be found in [this folder.](https://github.com/AREVD/Phantom/tree/main/Fisheye%20Calibration%20Core) The output files of stereo.py are used to perform stereo camera calibration when running AR.py.

*Note: Some calibration images were post-processed to improve image clarity for the checkerboard recognition stage of camera calibration.*

### Notes

+ The location of the foramen of Monro is specified in the `triangulate` function as the variables `pointA` and `pointB`. These values can be adjusted if necessary.
+ The tip of the catheter is colored green in our development, and the code reflects this in lines 153-158 (see below). 
  These RGB 
  value thresholds are responsible for identifying the 2D location of the tip of the catheter in each camera view by identifying green pixels and finding the average of the x and y coordinates of the pixels. These thresholds can be adjusted to handle differences in lighting conditions and/or different color choices for the catheter tip.
```
if buf is not None:
    if item == "A":
        lower = [14, 110, 0]  # color green boundaries
        upper = [70, 255, 156]
    elif item == "C":
        lower = [14, 140, 0]  # color green boundaries
        upper = [70, 255, 156]
```
+ Manual offsets in line 190 can be adjusted to accommodate differences in observed distance values, which may result 
  from variations between individual cameras.
```
c = str('%.2f' % (abs(6.6 * round(triangulate(mtxA, mtxB, R, T, ptA, ptC), 2) - 1.7)))
```
+ The brightness level is set using pulse width modulation (PWM), as full brightness can often make the 
  identification of the catheter tip color difficult due to over-exposure. This results in visible flickering of the 
  LED, which is expected. The brightness level is set at 25% in lines 103-104 (see below), and can be adjusted as 
  necessary.
```
p = gp.PWM(32,25)
p.start(25)
```
+ We set up a wireless network communication system using multithreading to send these distance results in real time to 
  our AR 
  headset, the Microsoft 
  HoloLens 2. This implementation was omitted from the Quick Start script for simplicity as it is highly tailored to 
  our 
  experimental setup.

### Associated Demo
The associated demo, Did You Do Well? Real-Time Personalized Feedback on Catheter Placement in Augmented Reality-assisted Neurosurgical Training, presents an AR-assisted neurosurgical training tool that provides real-time personalized feedback based on trainees' manipulation of the surgical environment and eye gaze patterns. The video of this demo can be found on [YouTube](https://youtu.be/AKNKKrCvapI). 

### Citation
Please cite the following paper in your publications if this code helps your research.
```
@inproceedings{eom2022ar,
  title={Did I Do Well? Personalized Assessment of Trainees' Performance in Augmented Reality-assisted Neurosurgical Training},
  author={Eom, Sangjun and Ma, Tiffany and Hu, Tianyi and Vutakuri, Neha and Jackson, Joshua and Gorlatova, Maria},
  booktitle={2024 IEEE Conference on Virtual Reality and 3D User Interfaces Abstracts and Workshops (VRW)},
  year={2024},
  organization={IEEE}
}
```
### Acknowledgments
The contributors of the code are [Tiffany Ma](https://sites.duke.edu/tiffanyma/), [Sangjun Eom](https://sites.duke.edu/sangjuneom/), and [Maria Gorlatova](https://maria.gorlatova.com/). For questions on this repository or the related paper, please contact Sangjun (Sarah) Eom at sangjun.eom [AT] duke [DOT] edu.
This work was supported in part by NSF grants CNS-2112562 and CNS-1908051, NSF CAREER Award IIS-2046072, and by a Thomas Lord Educational Innovation Grant.
