
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    loadCalibrationData(P_rect_00, R_rect_00, RT);

    // misc
    double sensorFrameRate = 10.0; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    bool bVis = false;            // visualize results

    vector<string> Detectors={"SHITOMASI"};//{"SHITOMASI", "HARRIS" ,"SIFT", "FAST", "BRISK", "ORB", "AKAZE"};
    vector<string> Descriptors={"SIFT", "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE"};
    string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
    /* MAIN LOOP OVER ALL IMAGES */
    map<int, double> ttc_Lidar;
    string ttc_stat = matcherType + "_" + selectorType +".csv";
    ofstream ttc(ttc_stat);
    for(auto detType: Detectors) //Loop Over Detector
    {
        for(auto descType: Descriptors) //Loop Over Descritpor
        {
            map<int, double> ttc_Camera;
            cout<<"***************************************************Detector: "<<detType<<"  ***********descriptor: "<<descType;
            if(detType.compare("SIFT")==0 && descType.compare("ORB")==0) continue;
            if(detType.compare("AKAZE")!=0 && descType.compare("AKAZE")==0) continue;

            vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
            {
                /* LOAD IMAGE INTO BUFFER */
                cout<<"***image id:"<<imgIndex<<endl;
                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                // load image from file
                cv::Mat img = cv::imread(imgFullFilename);

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = img;
                dataBuffer.push_back(frame);

                cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


                /* DETECT & CLASSIFY OBJECTS */

                //float confThreshold = 0.2;
                float confThreshold = 0.2;
                float nmsThreshold = 0.4;
                bVis = false;
                detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                              yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);
                bVis = false;
                cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;


                /* CROP LIDAR POINTS */

                // load 3D Lidar points from file
                string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
                vector<LidarPoint> lidarPoints;
                loadLidarFromFile(lidarPoints, lidarFullFilename);

                // remove Lidar points based on distance properties
                float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
                cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

                (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

                cout << "#3 : CROP LIDAR POINTS done" << endl;


                /* CLUSTER LIDAR POINT CLOUD */

                // associate Lidar points with camera-based ROI
                float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
                clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

               //Visualize 2D objects
               bVis = false;
               if(bVis)
               {
                    project_lidar_to_camera(*(dataBuffer.end()-1), P_rect_00, R_rect_00, RT,bVis);
               }
               bVis = false;

                // Visualize 3D objects
                bVis = true;
                if(bVis)
                {
                    show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(1000, 1000), true);
                }
                bVis = false;

                cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;


                // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
                //continue; // skips directly to the next image without processing what comes beneath

                /* DETECT IMAGE KEYPOINTS */


                // convert current image to grayscale
                cv::Mat imgGray;
                cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                string detectorType = detType;//{"SHITOMASI", "HARRIS","SIFT", "FAST", "BRISK", "ORB", "AKAZE"}

                detKeypointsModern(keypoints, imgGray, detectorType, false);

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;

                cout << "#5 : DETECT KEYPOINTS done" << endl;


                /* EXTRACT KEYPOINT DESCRIPTORS */

                cv::Mat descriptors;
                string descriptorType = descType; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
                descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                cout << "#6 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string descriptorCategory= descType.compare("SIFT")==0? "DES_HOG":"DES_BINARY";
                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                     (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                     matches, descriptorCategory, matcherType, selectorType);

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;


                    /* TRACK 3D OBJECT BOUNDING BOXES */

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
                    map<int, int> bbBestMatches;
                    matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1),false); // associate bounding boxes between current and previous frame using keypoint matches
                    for(auto it = bbBestMatches.begin();it!=bbBestMatches.end();++it)
                    {
                        cout<<"Current box: "<<it->first<<" vs Previous box: "<<it->second<<endl;
                    }
                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end()-1)->bbMatches = bbBestMatches;

                    cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;


                    /* COMPUTE TTC ON OBJECT IN FRONT */

                    // loop over all BB match pairs
                    for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
                    {
                        // find bounding boxes associates with current match
                        BoundingBox *prevBB, *currBB;
                        for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                        {
                            if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                            {
                                currBB = &(*it2);
                            }
                        }

                        for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                        {
                            if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                            {
                                prevBB = &(*it2);
                            }
                        }
                        //cout<<"*********************currBB ******"<<currBB->boxID<<"***size***"<<currBB->lidarPoints.size()<<endl;
                        //cout<<"*********************prevBB ******"<<prevBB->boxID<<"***size***"<<prevBB->lidarPoints.size()<<endl;
                        // compute TTC for current match
                        if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                        {
                            //// STUDENT ASSIGNMENT
                            //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                            double ttcLidar;
                            computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                            if(ttc_Lidar.find(imgIndex)==ttc_Lidar.end())
                            {
                                ttc_Lidar.insert(make_pair(imgIndex,ttcLidar));
                            }
                            //// EOF STUDENT ASSIGNMENT

                            //// STUDENT ASSIGNMENT
                            //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                            //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                            double ttcCamera;
                            //clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints
                            //                                , (dataBuffer.end() - 1)->keypoints
                            //                                , (dataBuffer.end() - 1)->kptMatches);

                            computeTTCCamera((dataBuffer.end() - 2)->keypoints
                                           , (dataBuffer.end() - 1)->keypoints
                                           , currBB->kptMatches
                                           , sensorFrameRate, ttcCamera);
                            //// EOF STUDENT ASSIGNMENT
                            ttcCamera = ttcCamera < 0? 0:ttcCamera;
                            ttcCamera = ttcCamera > 100? 100:ttcCamera;
                            ttc_Camera.insert(make_pair(imgIndex, ttcCamera));
                            cout<<"*********************************TTC Camera: "<<ttcCamera<<endl;
                            bVis = true;
                            if (bVis)
                            {
                                cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                                showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                                cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

                                char str[200];
                                sprintf(str, "Image ID: %d,  TTC Lidar : %2.3f s, TTC Camera : %2.3f s", (int)imgIndex, ttcLidar, ttcCamera);
                                putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));

                                string windowName = "Final Results : TTC";
                                cv::namedWindow(windowName, 4);
                                cv::imshow(windowName, visImg);
                                cout << "Press key to continue to next frame" << endl;
                                cv::waitKey(1);
                            }
                            bVis = false;
                        } // eof TTC computation
                    } // eof loop over all BB matches
                }
            } // eof loop over all images

            ttc << detType<<"_"<<descType;

            for (size_t imgIndex = 1; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
            {
                if(ttc_Camera.find(imgIndex)==ttc_Camera.end())
                {
                    ttc<< "," <<"inf";
                }else{
                    ttc<< "," <<ttc_Camera[imgIndex];
                }
            }
            ttc<<endl;
        }
    }
    ttc << "Lidar";

    for (size_t imgIndex = 1; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth)
    {
        if(ttc_Lidar.find(imgIndex)==ttc_Lidar.end())
        {
            ttc<< "," <<"inf";
        }else{
            ttc<< "," <<ttc_Lidar[imgIndex];
        }
    }
    ttc<<endl;
    ttc.close();
    cout<<endl;
    return 0;
}
