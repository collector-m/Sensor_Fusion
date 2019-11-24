void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    typedef std::function<bool(std::pair<int, int>, std::pair<int, int>)> Comparator;
    Comparator compFunctor =
  			[](std::pair<int, int> elem1 ,std::pair<int, int> elem2)
  			{
  				return elem1.second > elem2.second;
  			};

    // ...
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
    std::set<std::pair<int, int>, Comparator> setOfBox(
    tmpMatch.begin(), tmpMatch.end(), compFunctor);
    if(setOfBox.begin()->second>100)
      bbBestMatches.insert(pair<int,int>(it1->boxID, setOfBox.begin()->first));

      cv::Mat visImg1 = currFrame.cameraImg.clone();
      cv::Mat visImg2 = prevFrame.cameraImg.clone();
      for(auto it = it1->kptMatches.begin();it!=it1->kptMatches.end();++it)
      {
        circle(visImg1, Point(currFrame.keypoints[it->trainIdx].pt.x,
                             currFrame.keypoints[it->trainIdx].pt.y),2,Scalar(0,255,0));
      }
      int tmpID;
      for(auto it = prevFrame.boundingBoxes.begin();
                                       it!=prevFrame.boundingBoxes.end();++it)
      {
        if(it->boxID == setOfBox.begin()->first && setOfBox.begin()->second>4)
        {      cv::rectangle(visImg2, Point(it->roi.x, it->roi.y)
                          ,Point(it->roi.x+it->roi.width, it->roi.y+it->roi.height)
                          ,Scalar(0,255,0) );;break;}
      }
      cv::rectangle(visImg1, Point(it1->roi.x, it1->roi.y)
                  ,Point(it1->roi.x+it1->roi.width, it1->roi.y+it1->roi.height)
                   ,Scalar(0,255,0) );

      //hconcat(visImg1,visImg2);
      string windowName = "First";
      cv::namedWindow(windowName, 2);
      cv::imshow(windowName, visImg1);
      waitKey(0);
      windowName = "Second";
      cv::namedWindow(windowName, 2);
      cv::imshow(windowName, visImg2);
      waitKey(0);
    }


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
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


  for(auto it= bbBestMatches.begin();it!=bbBestMatches.end();++it)
  {
    cv::Mat visImg1 = currFrame.cameraImg.clone();
    cv::Mat visImg2 = prevFrame.cameraImg.clone();
    for(auto it_box = currFrame.boundingBoxes.begin();
             it_box!=currFrame.boundingBoxes.end();++it_box)
    {
      if(it_box->boxID == it->first)
      {
        cv::rectangle(visImg1, Point(it_box->roi.x, it_box->roi.y)
                    ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                     ,Scalar(0,255,0) );
        break;
      }
    }
    for(auto it_box = prevFrame.boundingBoxes.begin();
             it_box!= prevFrame.boundingBoxes.end();++it_box)
    {
      if(it_box->boxID == it->second)
      {
        cv::rectangle(visImg2, Point(it_box->roi.x, it_box->roi.y)
                    ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                     ,Scalar(0,255,0) );
        break;
      }
    }
    string windowName = "First";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, visImg1);
    waitKey(0);
    windowName = "Second";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, visImg2);
    waitKey(0);
  }

  void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
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
            //it1->kptMatches.push_back(*it2);
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

      if(bestMatchValue>40)
        bbBestMatches.insert(pair<int,int>(it1->boxID, bestMatchBox));
        std::vector<BoundingBox>::iterator it3;
        for( it3 = prevFrame.boundingBoxes.begin();
                 it3!=prevFrame.boundingBoxes.end();++it3)
        {
          if(it3->boxID == bestMatchBox)
          break;
        }
        for(auto it2 = matches.begin();it2!=matches.end();++it2)
        {
          cv::Point pt1;
          pt1.x = currFrame.keypoints[it2->trainIdx].pt.x;
          pt1.y = currFrame.keypoints[it2->trainIdx].pt.y;
          cv::Point pt2;
          pt2.x = prevFrame.keypoints[it2->queryIdx].pt.x;
          pt2.y = prevFrame.keypoints[it2->queryIdx].pt.y;
          if(it1->roi.contains(pt1) && it3->roi.contains(pt2))
          {
            it1->kptMatches.push_back(*it2);
            it1->keypoints.push_back(pt1);
            it3->keypoints.push_back(pt2);
          }
        }

    }

    for(auto it= bbBestMatches.begin();it!=bbBestMatches.end();++it)
    {
      cv::Mat visImg1 = currFrame.cameraImg.clone();
      cv::Mat visImg2 = prevFrame.cameraImg.clone();
      vector<cv::DMatch> tmpmatches;
      for(auto it_box = currFrame.boundingBoxes.begin();
               it_box!=currFrame.boundingBoxes.end();++it_box)
      {
        if(it_box->boxID == it->first)
        {
          cv::rectangle(visImg1, Point(it_box->roi.x, it_box->roi.y)
                      ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                       ,Scalar(0,255,0) );
          tmpmatches = it_box->kptMatches ;
          break;
        }
      }
      for(auto it_box = prevFrame.boundingBoxes.begin();
               it_box!= prevFrame.boundingBoxes.end();++it_box)
      {
        if(it_box->boxID == it->second)
        {
          cv::rectangle(visImg2, Point(it_box->roi.x, it_box->roi.y)
                      ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                       ,Scalar(0,255,0) );
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
      cv::namedWindow(windowName, 2);
      cv::imshow(windowName, matchImg);
      waitKey(0);/*
      windowName = "Second";
      cv::namedWindow(windowName, 2);
      cv::imshow(windowName, visImg2);
      waitKey(0);*/
    }
  }

  void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
  {
      for(auto it1 = currFrame.boundingBoxes.begin();
               it1!=currFrame.boundingBoxes.end();++it1)
      { //allocate mateched key points to each boundingBox
        map<int,int> tmpMatchedBoxes; //map<pre_BoxID, # of matched KeyPoint>
        for(auto it2 = matches.begin();it2!=matches.end();++it2)  //loop through all matched points
        {
          cv::KeyPoint currPt =currFrame.keypoints[it2->trainIdx] ;
          if(it1->roi.contains(currPt.pt)){
            //it1->kptMatches.push_back(*it2);
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

      if(bestMatchValue>10) //filte
        bbBestMatches.insert(pair<int,int>(it1->boxID, bestMatchBox));
        std::vector<BoundingBox>::iterator it3;
        for( it3 = prevFrame.boundingBoxes.begin();
                 it3!=prevFrame.boundingBoxes.end();++it3)
        {
          if(it3->boxID == bestMatchBox)
          break;
        }
        for(auto it2 = matches.begin();it2!=matches.end();++it2)
        {
          cv::KeyPoint currPt =currFrame.keypoints[it2->trainIdx] ;
          cv::KeyPoint prevPt =prevFrame.keypoints[it2->queryIdx] ;

          if(it1->roi.contains(currPt.pt) && it3->roi.contains(prevPt.pt))
          {
            it1->kptMatches.push_back(*it2);
            it1->keypoints.push_back(currPt);
            it3->keypoints.push_back(prevPt);
          }
        }

    }

    for(auto it= bbBestMatches.begin();it!=bbBestMatches.end();++it)
    {
      cv::Mat visImg1 = currFrame.cameraImg.clone();
      cv::Mat visImg2 = prevFrame.cameraImg.clone();
      vector<cv::DMatch> tmpmatches;
      for(auto it_box = currFrame.boundingBoxes.begin();
               it_box!=currFrame.boundingBoxes.end();++it_box)
      {
        if(it_box->boxID == it->first)
        {
          cv::rectangle(visImg1, Point(it_box->roi.x, it_box->roi.y)
                      ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                       ,Scalar(0,255,0) );
          tmpmatches = it_box->kptMatches ;
          break;
        }
      }
      for(auto it_box = prevFrame.boundingBoxes.begin();
               it_box!= prevFrame.boundingBoxes.end();++it_box)
      {
        if(it_box->boxID == it->second)
        {
          cv::rectangle(visImg2, Point(it_box->roi.x, it_box->roi.y)
                      ,Point(it_box->roi.x+it_box->roi.width, it_box->roi.y+it_box->roi.height)
                       ,Scalar(0,255,0) );
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
      cv::namedWindow(windowName, 2);
      cv::imshow(windowName, matchImg);
      waitKey(0);/*
      windowName = "Second";
      cv::namedWindow(windowName, 2);
      cv::imshow(windowName, visImg2);
      waitKey(0);*/
    }
  }
