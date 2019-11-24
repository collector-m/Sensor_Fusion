#include <numeric>
#include "matching2D.hpp"
#include <stdexcept>
#include <map>

using namespace std;
using namespace cv;

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = true;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    if (matcherType.compare("MAT_BF") == 0)
    {
        if (selectorType.compare("SEL_KNN") == 0) crossCheck = false;
        int normType = descriptorType.compare("DES_BINARY")==0? cv::NORM_HAMMING: cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout<<"Brute Force Matching" <<endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
      if (descSource.type() != CV_32F || descRef.type() != CV_32F)
      { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
          descSource.convertTo(descSource, CV_32F);
          descRef.convertTo(descRef, CV_32F);
      }
      matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      cout << "FLANN matching"<<endl;        // ...
    }
    double t = (double)cv::getTickCount();
    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    }
    return 1000 * t / 1.0;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{   //BRIEF, ORB, FREAK, AKAZE, SIFT
    // select appropriate descriptor

    cv::Ptr<cv::DescriptorExtractor> extractor;


    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType.compare("BRIEF") == 0)
    {
      //cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> extractor;
      extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(64);

    }else if(descriptorType.compare("ORB") == 0)
    {
    //  cv::Ptr<cv::ORB> extractor;
      extractor = cv::ORB::create();
    }else if(descriptorType.compare("FREAK") == 0)
    {
      //cv::Ptr<cv::xfeatures2d::FREAK> extractor;
      extractor = cv::xfeatures2d::FREAK::create();
    }else if(descriptorType.compare("AKAZE") == 0)
    {
      //cv::Ptr<cv::xfeatures2d::FREAK> extractor;
      extractor = cv::AKAZE::create();
    }else
    {
      extractor = cv::xfeatures2d::SIFT::create();
    //cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
  }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    return 1000 * t / 1.0;
}
// Detect keypoints in image using the moderm detector
double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{ //HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
  if(detectorType.compare("SHITOMASI")==0)
  {
    return detKeypointsShiTomasi(keypoints, img, bVis);
  }else  if(detectorType.compare("HARRIS")==0)
  {
    return detKeypointsHarris(keypoints, img, bVis);
  }else if(detectorType.compare("SIFT")==0)
  {
    return detKeypointsSIFT(keypoints, img, bVis);
  }else if(detectorType.compare("FAST")==0)
  {
   return detKeypointsFAST(keypoints, img, bVis);
   }else if(detectorType.compare("BRISK")==0)
   {
     return detKeypointsBRISK(keypoints, img, bVis);
   }else if(detectorType.compare("ORB")==0)
   {
     return detKeypointsORB(keypoints, img, bVis);
   }else if(detectorType.compare("AKAZE")==0)
   {
     return detKeypointsAKAZE(keypoints, img, bVis);
   }
}


double detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

  cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
  double t = (double)cv::getTickCount();
  detector->detect(img,keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  return 1000 * t / 1.0;
}

double detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  int nFeatures = 100;
  int nLevels = 8;
  float scaleFactor = 1.2f;
  //cv::Ptr<cv::ORB> detector = cv::ORB::create(nFeatures, scaleFactor, nLevels);
  cv::Ptr<cv::ORB> detector = cv::ORB::create();
  double t = (double)cv::getTickCount();
  detector->detect(img,keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  return 1000 * t / 1.0;
}

double detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  int Thresh = 100;
  int Octave = 4;
  float PatternScales = 1.0f;
  //cv::Ptr<cv::BRISK> detector = cv::BRISK::create(Thresh, Octave, PatternScales);
  cv::Ptr<cv::BRISK> detector = cv::BRISK::create();
  double t = (double)cv::getTickCount();
  detector->detect(img,keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  return 1000 * t / 1.0;
}

double detKeypointsSURF(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  //int numFeatures = 400;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
  double t = (double)cv::getTickCount();
  detector->detect(img,keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "SIFT Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
  return 1000 * t / 1.0;
}

double detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  //int numFeatures = 400;
  cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
  double t = (double)cv::getTickCount();
  detector->detect(img,keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "SIFT Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
  return 1000 * t / 1.0;
}
double detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  int threshold = 30;       // difference between intensity of the central pixel and pixels of a circle around this pixel
  bool bNMS = true;
  cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
  cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
  double t = (double)cv::getTickCount();
  detector->detect(img, keypoints);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "FAST Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
  return 1000 * t / 1.0;
}
// Detect keypoints in image using the traditional Harris detector
double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  // Detector parameters
  int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
  int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
  int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
  double k = 0.04;       // Harris parameter (see equation for details)
  // Detect Harris corners and normalize output
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);
  double t = (double)cv::getTickCount();
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);
  // Look for prominent corners and instantiate keypoints
  //vector<cv::KeyPoint> keypoints;
  double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
  for (size_t j = 0; j < dst_norm.rows; j++)
  {
      for (size_t i = 0; i < dst_norm.cols; i++)
      {
          int response = (int)dst_norm.at<float>(j, i);
          if (response > minResponse)
          { // only store points above a threshold

              cv::KeyPoint newKeyPoint;
              newKeyPoint.pt = cv::Point2f(i, j);
              newKeyPoint.size = 2 * apertureSize;
              newKeyPoint.response = response;

              // perform non-maximum suppression (NMS) in local neighbourhood around new key point
              bool bOverlap = false;
              for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
              {
                  double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                  if (kptOverlap > maxOverlap)
                  {
                      bOverlap = true;
                      if (newKeyPoint.response > (*it).response)
                      {                      // if overlap is >t AND response is higher for new kpt
                          *it = newKeyPoint; // replace old key point with new one
                          break;             // quit loop over keypoints
                      }
                  }
              }
              if (!bOverlap)
              {                                     // only add new key point if no overlap has been found in previous NMS
                  keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
              }
          }
      } // eof loop over cols

  }     // eof loop over rows
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  // visualize results
  if (bVis)
  {
    string windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 6);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);
  }
  return 1000 * t / 1.0;
}
// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
  return 1000 * t / 1.0;
}
