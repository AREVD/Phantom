# Camera Sensor-Embedded Phantom Model for External Ventricular Drain
This repository contains the developmental artifacts for the custom phantom skull model used in the paper "Did I Do Well? Instantaneous Assessment of Trainees' Performance in Augmented Reality-assisted Neurosurgical Training", submitted to **IEEE VR 2024** for review, and includes all of the relevant code and assets necessary for the implementation of real-time distance calculation.

# Outline
1. [Overview](https://github.com/AREVD/Phantom/blob/main/README.md#overview)
2. [Hardware Setup](https://github.com/AREVD/Phantom/blob/main/README.md#hardware_setup)
3. [Raspberry Pi Software Setup]
4. [Implementation]

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
1. Acrylic Sheets
2. [CHANZON White LED Diode Lights](https://a.co/d/ftPlEu8)
3. [Custom Collagen SuperClear Gelatin](https://customcollagenshop.com/products/superclear-gelatin)

### Additional Tools and Materials
1. Hot glue + hot glue gun
2. Nuts and bolts for securing cameras onto camera mounts
3. Electrical tape for securing camera mounts to gelatin mold
4. (2x) Female to female DuPont wires (~50cm)
5. Laser cutter or precision knife
6. [CODMAN BACTISEAL EVD Catheter Set](https://www.integralife.com/codman-bactiseal-evd-catheter-set/product/neurocritical-care-evd-catheters-bactiseal-anti-microbial-catheter-codman-bactiseal-evd-catheter-set)
7. Colored polish/paint or silicone dye

All 3D models were printed using PLA filament and Ultimaker 3 printers. Black filament was used to print the gelatin mold and camera mounts to create a high-contrast environment for the cameras, and light grey filament was used to print the phantom skull to present a non-distracting visual experience to the user.

## Camera Hardware Setup

Required 3D print: [Camera_mounts.stl](https://github.com/AREVD/Phantom/3D_Models/Camera_mounts.stl)

1. Replace cameras' native cables with the 60cm extension cables.
2. Secure cameras to camera mounts using nuts and bolts on the four corners of each camera.
3. Connect cameras to the multi camera adapter using the extension cables.
4. Mount the adapter to pins 1-26 of the Raspberry Pi. Secure with nuts and bolts if desired.
5. Connect the adapter's camera cable to the Raspberry Pi.

## Gelatin Mold Setup

Required 3D print: [Gelatin_mold.stl](https://github.com/AREVD/Phantom/3D_Models/Gelatin_mold.stl)

1. Cut acrylic sheets into 2-inch squares. Two squares are needed per mold.
2. Secure squares from the inside of the gelatin mold using hot glue along all edges of the squares.
3. Secure LED diode light to the bottom corner between the two camera windows using hot glue.
4. Ensure the mold is water-tight - if not, add hot glue as necessary.
5. Connect the LED diode light to pins 32 (GPIO 12) and 34 (Ground) using DuPont wires. 

*Note: For troubleshooting regarding the multi camera adapter, please refer to the [Arducam documentation of the product.](https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/Multi-Camera-CamArray/#arducam-multi-camera-adapter-board)*

## Gelatin Mixture Instructions

The following steps yield three molds' worth of gelatin mixture.

1. Weigh out 20g of gelatin and combine with 250ml of cold water. Mix until clumping is no longer visualized. Let sit for 10-15 minutes.
2. Heat 700ml of water until steam is visualized. Closely monitor and do not allow to boil. A microwave is sufficient for heating.
3. Combine the gelatin mixture with the larger volume of water. Mix thoroughly until mixture is clear.
4. Cool mixture to room temperature. This can be accelerated by refrigerating the mixture. *Note: Closely monitor to prevent excessive cooling, which would cause the gelatin to coagulate prematurely.*
5. Disconnect the gelatin molds' LED diode light from the DuPont wires.
6. Suspend the molds in a cup or other container using paper towels or other supports such that the circular opening faces upwards and is parallel to the ground.
7. Fill the molds with the mixture up to the opening.
8. Refrigerate for at least 1 hour.

## Final Hardware Setup

Required 3D prints: [Top_skull_pegs.stl](https://github.com/AREVD/Phantom/3D_Models/Top_skull_pegs.stl), [Bottom_skull_pegs.stl](https://github.com/AREVD/Phantom/3D_Models/Bottom_skull_pegs.stl)

1. Thread the DuPont wires through the holes in the bottom of the phantom into the inside of the phantom.
2. Connect the DuPont wires to the LED diode light of the gelatin mold.
3. Secure the camera mounts onto the gelatin mold using electrical tape.
4. Insert the mold into the peg in the center of the phantom.
5. Close the phantom by connecting the top of the skull to the bottom of the skull. Secure with tape if necessary.
