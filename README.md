# Co-Fusion

This repository contains Co-Fusion, a dense SLAM system that takes a live stream of RGB-D images as input and segments the scene into different objects.

Crucially, we use a multiple model fitting approach where each object can move independently from the background and still be effectively tracked and its shape fused over time using only the information from pixels associated with that object label. Previous attempts to deal with dynamic scenes have typically considered moving regions as outliers that are of no interest to the robot, and consequently do not model their shape or track their motion over time. In contrast, we enable the robot to maintain 3D models for each of the segmented objects and to improve them over time through fusion. As a result, our system has the benefit to enable a robot to maintain a scene description at the object level which has the potential to allow interactions with its working environment; even in the case of dynamic scenes.

To run Co-Fusion in real-time, you have to use our approach based no motion cues. If you prefer to use semantic cues for segmentation, please pre-process the segmentation in advance and feed the resulting segmentation masks into Co-Fusion.

More information and the paper can be found [here](http://visual.cs.ucl.ac.uk/pubs/cofusion/index.html).

## Publication

Please cite this publication, when using Co-Fusion (bibtex can be found on project webpage):

* Co-Fusion: **Real-time Segmentation, Tracking and Fusion of Multiple Objects**, Martin Rünz and Lourdes Agapito, 2017 IEEE International Conference on Robotics and Automation (ICRA)

## Building Co-Fusion

The script `Scripts/install.sh` shows step-by-step how Co-Fusion is build. A python-based install script is also available, see `Scripts\install.py`.

## Dataset and evaluation tools

### Tools
* Recorder for klg files: https://github.com/mp3guy/Logger2
* Viewer for klg files: https://github.com/mp3guy/LogView
* Images -> klg converter: https://github.com/martinruenz/dataset-tools/tree/master/convert_imagesToKlg
* klg -> images/pointclouds: https://github.com/martinruenz/dataset-tools/tree/master/convert_klg
* Evaluate segmentation (intersection-over-union): https://github.com/martinruenz/dataset-tools/tree/master/evaluate_segmentation
* Automated evaluation of Co-Fusion: https://github.com/martinruenz/dataset-tools/tree/master/automatisation
* Scripts to create synthetic datasets with blender: https://github.com/martinruenz/dataset-tools/tree/master/blender

### Synthetic sequences:
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/car4-noise.klg
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/car4-full.tar.gz
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/room4-noise.klg
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/room4-full.tar.gz

### Real (Asus Xtion) sequences, in [klg](https://github.com/mp3guy/LogView) format:
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/teddy-handover.klg
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/sliding-clock.klg
* http://visual.cs.ucl.ac.uk/pubs/cofusion/data/place-items.klg

## Hardware

In order to run Co-Fusion smoothly, you need a fast GPU with enough memory to store multiple models simultaneously. We used an Nvidia TitanX for most experiments, but also successfully tested Co-Fusion on a laptop computer with an Nvidia GeForce™ GTX 960M. If your GPU memory is limited, the `COFUSION_NUM_SURFELS` CMake option can help reduce the memory footprint per model.
While the tracking stage of Co-Fusion calls for a fast GPU, the motion based segmentation performance depends on the CPU and accordingly, having a nice processor helps as well.

## Reformatting code:
The code-formatting rules for this project are defined `.clang-format`. Run:

    clang-format -i -style=file Core/**/*.cpp Core/**/*.h Core/**/*.hpp GUI/**/*.cpp GUI/**/*.h GUI/**/*.hpp

## ElasticFusion
The overall architecture and terminal-interface of Co-Fusion is based on [ElasticFusion](https://github.com/mp3guy/ElasticFusion) and the ElasticFusion [readme file](https://github.com/mp3guy/ElasticFusion/blob/master/README.md) contains further useful information.

## New command line parameters (see [source-file](https://github.com/martinruenz/co-fusion/blob/master/GUI/MainController.cpp#L29-L90))

* **-run**:           Run dataset immediately (otherwise start paused).
* **-static**:        Disable multi-model fusion.
* **-confO**:         Initial surfel confidence threshold for objects (default 0.01).
* **-confG**:         Initial surfel confidence threshold for scene (default 10.00).
* **-segMinNew**:     Min size of new object segments (relative to image size)
* **-segMaxNew**:     Max size of new object segments (relative to image size)
* **-offset**:        Offset between creating models
* **-keep**:          Keep all models (even bad, deactivated)
* **-dir**:           Processes a log-directory (Default: Color####.png + Depth####.exr [+ Mask####.png])
* **-depthdir**:      Separate depth directory (==dir if not provided)
* **-maskdir**:       Separate mask directory (==dir if not provided)
* **-exportdir**:     Export results to this directory, otherwise not exported
* **-basedir**:       Treat the above paths relative to this one (like depthdir = basedir + depthdir, default "")
* **-colorprefix**:   Specify prefix of color files (=="" or =="Color" if not provided)
* **-depthprefix**:   Specify prefix of depth files (=="" or =="Depth" if not provided)
* **-maskprefix**:    Specify prefix of mask files (=="" or =="Mask" if not provided)
* **-indexW**:        Number of digits of the indexes (==4 if not provided)
* **-nm**:            Ignore Mask####.png images as soon as the provided frame was reached.
* **-es**:            Export segmentation
* **-ev**:            Export viewport images
* **-el**:            Export label images
* **-em**:            Export models (point-cloud)
* **-en**:            Export normal images
* **-ep**:            Export poses after finishing run (just before quitting if '-q')
* **-or**:            Outlier rejection strength (default 3).
