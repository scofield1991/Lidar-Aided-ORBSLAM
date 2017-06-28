/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include<mutex>

#include "Odom/NavState.h"

extern cv::Mat K;
namespace ORB_SLAM2
{

class KeyFrame;
class Frame;
class Map;

struct KFItem
{
    cv::Mat Twc;
    NavState Twl;
    int score;
    DBoW2::BowVector vBow;
    bool operator<( const KFItem& other ) const
    {
        return ((Twl.Get_P() - other.Twl.Get_P()).norm() < 1e-5) && (score == other.score) && (vBow.size() == other.vBow.size());
    }
};

class KeyFrameDatabase
{
  
public:
  
  void add(KFItem pKFI);
  void add(ModelKeyFrame &mMdKF);

  void erase(KFItem pKFI);
  void erase(ModelKeyFrame &mMdKF);
  
  // Detect init localization
  std::vector<KFItem> DetectInitLocalization(Frame* F);
  std::vector<ModelKeyFrame> SelectCandidates(Frame* F);
  
protected:
  // Inverted file
  std::vector<list<KFItem> > mvInvPoseFile;
  std::vector<list<ModelKeyFrame>> mvInvModelKF;
  
public:

   KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
