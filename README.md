# RTES Project

This is the final project for CU Boulder's Real-Time Embedded Systems
specialization.

This project:
- Uses Linux's V4L2 framework to capture images of an analog/digital clock using
  a web camera
- Identifies each second/one-tenth of a second transition in the respective
  clock and selects the most stable frame
- Includes the option to process the selected frames using a Sorbel filter
- Converts the selected/modified frames from YUYV to RGB format
- Writes the selected/modified frames to disk

## Requirements

- Use a web camera to capture and save frames of an analog clock at a rate of 1
  Hz and a digital clock at a rate of 10 Hz.
- The captured frames must have a frame size of at least 320x240 pixels.
- There must be 1800+1 captured and saved frames with no repeats, skips, or
  blurs.
- There must be an option to implement an image processing algorithm on the
  captured and saved frames.
- The project must have at least two real-time services running on a single
  core.

## Data Flow Diagram

<img src="images/data-flow-diagram.png">

## Software Flowchart

<img src="images/flowchart.png">

## Real-Time Requirements

| Service   | WCET (ms) | T (ms)    | D (ms)    | Core  |
| ---       | ---       | ---       | ---       | ---   |
| Read      | 0.25      | 33        | 33        | 2     | 
| Select    | 25        | 200       | 200       | 2     |
| Modify    | ??        | 500       | 500       | 2     |
| Write     | 65        | 1000      | 1000      | 2     |

$U = \frac{0.25}{33} + \frac{25}{200} + \frac{65}{1000} = 0.1976 = 19.76\%$

### Timing Diagram

## Real-Time Analysis of Results

### Timing Analysis

#### 1-Hz Capture

<img src="images/s1-1hz.png">

<img src="images/s2-1hz.png">

<img src="images/s4-1hz.png">

#### 1-Hz Capture with Processing

<img src="images/s1-1hz-processed.png">

<img src="images/s2-1hz-processed.png">

<img src="images/s3-1hz-processed.png">

<img src="images/s4-1hz-processed.png">

#### 10-Hz Capture

<img src="images/s1-10hz.png">

<img src="images/s2-10hz.png">

<img src="images/s4-10hz.png">

#### 10-Hz Capture with Processing

<img src="images/s1-10hz-processed.png">

<img src="images/s2-10hz-processed.png">

<img src="images/s3-10hz-processed.png">

<img src="images/s4-10hz-processed.png">

### Jitter and Drift Analysis

<img src="images/jitter-1hz.png">

- $0.207688 - (-0.092216) = 0.29904 \approx 0.3 s$
- Jitter $\approx \pm 0.15 s$

<img src="images/drift-1hz.png">

- Drift $= 14.140 {\mu}s/frame \times 1 frame/s = 14.140 {\mu}s/s$

<img src="images/jitter-and-drift-10hz.png">

- $0.006566 s - 0 s = 0.006566 s$
- Jitter $\approx \pm 0.003283 s$
- Drift $\approx 2.043 {\mu}s/frame \times 10 frame/s = 20.431 {\mu}s/s$

<img src="images/jitter-and-drift-10hz-with-processing.png">

- Jitter $\approx \pm 0.04 s$
- Drift $\approx \frac{0.00505586 - 0.00219388}{1443 - 153}
  = 2.21859 {\mu}s/frame \times 10 frame/s = 22.186 {\mu}s/s$
