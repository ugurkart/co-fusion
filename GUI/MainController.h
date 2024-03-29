/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#pragma once

#include <CoFusion.h>
#include <Utils/Parse.h>

#include "Tools/GUI.h"
#include "Tools/GroundTruthOdometry.h"
#include "Tools/LogReader.h"

#include "Core/Socket/ClientSocket.h"
#include "Core/Socket/SocketException.h"
#define PORT 55000

class MainController
{
public:
    MainController(int argc, char* argv[]);
    virtual ~MainController();

    void launch();

    std::string getProjectionDir();

private:
    void run();

    enum DRAW_COLOR_TYPE { DRAW_USER_DEFINED = 0, DRAW_NORMALS = 1, DRAW_COLOR = 2, DRAW_TIMES = 3, DRAW_LABEL = 4 };
    void drawScene(DRAW_COLOR_TYPE backgroundColor = DRAW_USER_DEFINED, DRAW_COLOR_TYPE objectColor = DRAW_USER_DEFINED);
    void drawBoundingBox(std::vector<Eigen::Vector4f> drawn_vertices);
    void loadCalibration(const std::string& filename);
    void projectModel();

    bool good;
    CoFusion* coFusion;
    GUI* gui;
    bool showcaseMode;
    GroundTruthOdometry* groundTruthOdometry;
    std::unique_ptr<LogReader> logReader;

    bool iclnuim;
    std::string logFile;
    std::string poseFile;
    std::string exportDir;
    std::string project_dir;
    std::string center_file;
    std::string surfel_num_file;
    std::string icp_error_file;
    std::string all_icp_error_file;
    std::string euler_angles_file;
    bool exportSegmentation;
    bool exportViewport;
    bool exportLabels;
    bool exportNormals;
    bool exportPoses;
    bool exportModels;

    float confGlobalInit, confObjectInit, icpErrThresh, covThresh, photoThresh, fernThresh;

    int timeDelta, icpCountThresh, start, end, preallocatedModelsCount;

    bool fillIn, openLoop, reloc, frameskip, quit, fastOdom, so3, rewind, frameToFrameRGB;

    int framesToSkip;
    bool streaming;
    bool resetButton;

    GPUResize* resizeStream;

    ClientSocket* m_client_socket;
};
