
#include <iostream>
#include <algorithm>
#include <numeric>
#include <set>
#include <map>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;
using namespace cv;

void project_lidar_to_camera(DataFrame& frame,  cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT, bool bVis)
{
  cv::Mat visImg = frame.cameraImg.clone();
  cv::Mat overlay = visImg.clone();
  for(auto it=frame.boundingBoxes.begin(); it!=frame.boundingBoxes.end(); ++it)
  {
    int top, left, width, height;
    top = (*it).roi.y;
    left = (*it).roi.x;
    width = (*it).roi.width;
    height = (*it).roi.height;
    cv::rectangle(visImg, cv::Point(left, top), cv::Point(left+width, top+height),cv::Scalar(0, 255, 0), 2);
    string label = cv::format("%d", (*it).boxID);
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(visImg, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0,0,0),1);

  }
  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  for(auto it=frame.lidarPoints.begin(); it!=frame.lidarPoints.end(); ++it) {
      // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
      X.at<double>(0,0) = it->x;
      X.at<double>(1,0) = it->y;
      X.at<double>(2,0) = it->z;
      X.at<double>(3,0) = 1;
      // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera.
      Y =  P_rect_xx * R_rect_xx * RT * X;
      // Store the result in Y.

      // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
      cv::Point pt;
      pt.x = Y.at<double>(0,0)/Y.at<double>(0,2);
      pt.y = Y.at<double>(1,0)/Y.at<double>(0,2);

      float val = it->x;
      float maxVal = 20.0;
      int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
      int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
      cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);
  }
  float opacity = 0.6;
  cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

  string windowName = "Object classification";
  cv::namedWindow( windowName, 1 );
  cv::imshow( windowName, visImg );
  cv::waitKey(1); // wait for key to be pressed
}

void removeOutliers(std::vector<LidarPoint> &lidarPoints)
{
  sort(lidarPoints.begin(),lidarPoints.end());
  long medIdx = floor(lidarPoints.size()/2.0);
  long medianPtindex = lidarPoints.size()%2==0?medIdx -1 :medIdx;

  long Q1Idx = floor(medianPtindex/2.0);
  double Q1 = medianPtindex%2==0?(lidarPoints[Q1Idx -1].x +lidarPoints[Q1Idx].x)/2.0:lidarPoints[Q1Idx].x;
  auto it=lidarPoints.begin();
  while(it!=lidarPoints.end())
  {
    if(it->x < Q1)
    {
      it = lidarPoints.erase(it);
    }else{break;}
  }
}
// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    removeOutliers(lidarPoints);
    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }
        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0;
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            if(it2==it1->lidarPoints.begin())
            {
              //char str1[200], str2[200];
              //sprintf(str1, "reflect=%2.2f", it2->r);
              //putText(topviewImg, str1, cv::Point2f(left-150, bottom+250), cv::FONT_ITALIC, 1, cv::Scalar(0, 0, 255));
              cv::circle(topviewImg, cv::Point(x, y), 6, cv::Scalar(0, 0, 255), -1);
            }
            else
              cv::circle(topviewImg, cv::Point(x, y), 2, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(1); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
  vector<double> distRatios;
  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
  {
    cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
    cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);
    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
    {
      double minDist = 100.0; // min. required distance

      // get next keypoint and its matched partner in the prev. frame
      cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
      cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

      // compute distances and distance ratios
      double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
      double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

      if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
      { // avoid division by zero

          double distRatio = distCurr / distPrev;
          distRatios.push_back(distRatio);
      }
    }
  }
  if (distRatios.size() == 0)
  {
      TTC = NAN;
      return;
  }
  sort(distRatios.begin(), distRatios.end());
  long medIdx = floor(distRatios.size()/2.0);
  double medDistRatio = distRatios.size()%2==0?(distRatios[medIdx -1] +distRatios[medIdx])/2.0:distRatios[medIdx];
  double dT = 1 / frameRate;
  TTC = -dT / (1 - medDistRatio+1e-6);
  //double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
  //TTC = -1 / (1 - meanDistRatio)/frameRate;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double minXPrev = 1e9, minXCurr = 1e9;

    minXPrev = lidarPointsPrev.begin()->x;


    minXCurr = lidarPointsCurr.begin()->x;

    TTC = minXCurr  / (minXPrev - minXCurr+1e-6)/frameRate;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame, bool bVis)
{
    for(auto it1 = currFrame.boundingBoxes.begin();
             it1!=currFrame.boundingBoxes.end();++it1)
    {
      map<int,int> tmpMatchedBoxes; //map<pre_BoxID, # of matched KeyPoint>
      for(auto it2 = matches.begin();it2!=matches.end();++it2)  //loop through all matched points
      {
        cv::KeyPoint currPt =currFrame.keypoints[it2->trainIdx] ;
        if(it1->roi.contains(currPt.pt)){
          cv::KeyPoint prevPt =prevFrame.keypoints[it2->queryIdx] ;

          for(auto it3 = prevFrame.boundingBoxes.begin();
                   it3!=prevFrame.boundingBoxes.end();++it3)
          {
            if(it3->roi.contains(prevPt.pt))
            {
              int boxId = it3->boxID;
              auto it_tmp = tmpMatchedBoxes.find(boxId);
              if(it_tmp==tmpMatchedBoxes.end())
              {tmpMatchedBoxes.insert(make_pair(boxId, 1));}
              else{
                tmpMatchedBoxes[boxId] +=1;
              }
            }
          }
        }
      }
    int bestMatchBox = -1;
    int bestMatchValue = 0;
    for(auto it=tmpMatchedBoxes.begin();it!=tmpMatchedBoxes.end();++it)
    {
      if(it->second>bestMatchValue)
      {
        bestMatchBox = it->first;
        bestMatchValue = it->second;
      }
    }

    if(bestMatchValue>2) //filter out boxes which have only 1 or 2 matched points
    {
      bbBestMatches.insert(pair<int,int>(bestMatchBox,it1->boxID));
      std::vector<BoundingBox>::iterator it3;
      for( it3 = prevFrame.boundingBoxes.begin();
           it3 != prevFrame.boundingBoxes.end();++it3)
      {
        if(it3->boxID == bestMatchBox)
        break;
      }
      it1->kptMatches.clear();
      it1->keypoints.clear();
      it3->keypoints.clear();
      for(auto it2 = matches.begin();it2!=matches.end();++it2)
      {
        cv::KeyPoint currPt =currFrame.keypoints[it2->trainIdx] ;
        cv::KeyPoint prevPt =prevFrame.keypoints[it2->queryIdx] ;

        if(it1->roi.contains(currPt.pt)  && it3->roi.contains(prevPt.pt))
        {
          it1->kptMatches.push_back(*it2);
          it1->keypoints.push_back(currPt);
          it3->keypoints.push_back(prevPt);
        }
      }
    }
  }

if(bVis)
{
    for(auto it= bbBestMatches.begin();it!=bbBestMatches.end();++it)
    {
      cv::Mat visImg1 = currFrame.cameraImg.clone();
      cv::Mat visImg2 = prevFrame.cameraImg.clone();
      vector<cv::DMatch> tmpmatches;
      for(auto it_box = currFrame.boundingBoxes.begin();
               it_box!=currFrame.boundingBoxes.end();++it_box)
      {
        if(it_box->boxID == it->second)
        {
          string label = cv::format("%d", it_box->boxID);
          cv::rectangle(visImg1, Point(it_box->roi.x, it_box->roi.y)
                      ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                       ,Scalar(0,0,255),2 );
          cv::putText(visImg1, label, cv::Point(it_box->roi.x, it_box->roi.y), cv::FONT_ITALIC, 0.75, cv::Scalar(0,0,255),2);
          tmpmatches = it_box->kptMatches ;
          break;
        }
      }
      for(auto it_box = prevFrame.boundingBoxes.begin();
               it_box!= prevFrame.boundingBoxes.end();++it_box)
      {
        if(it_box->boxID == it->first)
        {
          string label = cv::format("%d", it_box->boxID);
          cv::rectangle(visImg2, Point(it_box->roi.x, it_box->roi.y)
                      ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height) ,Scalar(0,255,0),2 );
          cv::putText(visImg2, label, cv::Point(it_box->roi.x, it_box->roi.y), cv::FONT_ITALIC, 0.75, cv::Scalar(0,0,255),2);

          break;
        }
      }
      cv::Mat matchImg = visImg1.clone();
      cv::drawMatches(visImg2, prevFrame.keypoints,
                      visImg1, currFrame.keypoints,
                      tmpmatches, matchImg,
                      cv::Scalar::all(-1), cv::Scalar::all(-1),
                      vector<char>(), cv::DrawMatchesFlags::DEFAULT);


      string windowName = "First";
      cv::namedWindow(windowName, 0);
      cv::resizeWindow(windowName,2000,500);
      cv::imshow(windowName, matchImg);
      waitKey(0);

    }
  }
}

/*void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    for(auto it1 = currFrame.boundingBoxes.begin();
             it1!=currFrame.boundingBoxes.end();++it1)
    { //allocate mateched key points to each boundingBox
      map<int,int> tmpMatch;
      for(auto it2 = matches.begin();it2!=matches.end();++it2)
      {
        cv::Point pt;
        pt.x = currFrame.keypoints[it2->trainIdx].pt.x;
        pt.y = currFrame.keypoints[it2->trainIdx].pt.y;
        if(it1->roi.contains(pt)){
          it1->kptMatches.push_back(*it2);
          cv::Point pt;
          pt.x = prevFrame.keypoints[it2->queryIdx].pt.x;
          pt.y = prevFrame.keypoints[it2->queryIdx].pt.y;
          for(auto it3 = prevFrame.boundingBoxes.begin();
                   it3!=prevFrame.boundingBoxes.end();++it3)
          {
            if(it3->roi.contains(pt))
            {
              int boxId = it3->boxID;
              auto it_tmp = tmpMatch.find(boxId);
              if(it_tmp==tmpMatch.end())
              {tmpMatch.insert(make_pair(boxId, 1));}
              else{
                tmpMatch[boxId] +=1;
              }
            }
          }
        }
      }
    int bestMatchBox = -1;
    int bestMatchValue = 0;
    for(auto it=tmpMatch.begin();it!=tmpMatch.end();++it)
    {
      if(it->second>bestMatchValue)
      {
        bestMatchBox = it->first;
        bestMatchValue = it->second;
      }
    }

    if(bestMatchValue>10)
      bbBestMatches.insert(pair<int,int>(it1->boxID, bestMatchBox));
  }
}*/
void loadCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT)
{
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;

    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;

    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;

}
