#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "BoostArchiver.h"

#include<mutex>


namespace GPS_OFF_SLAM
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
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

// map serialization addition
public:
   // for serialization
   KeyFrameDatabase() {}
   void SetORBvocabulary(ORBVocabulary *porbv) {mpVoc=porbv;}
private:
   // serialize is recommended to be private
   friend class boost::serialization::access;
   template<class Archive>
   void serialize(Archive &ar, const unsigned int version);

};

} //namespace GPS_OFF_SLAM

#endif
