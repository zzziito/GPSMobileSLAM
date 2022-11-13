#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>

namespace GPS_OFF_SLAM
{

System::System(const string strVocFile, const eSensor sensor, ORBParameters& parameters,
               const std::string & map_file, bool load_map): // map serialization addition
               mSensor(sensor), mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
               map_file(map_file), load_map(load_map)
{

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else
        cout << "wrong sensor" <<endl;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary." << endl;

    mpVocabulary = new ORBVocabulary();

    //try to load from the binary file
    bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile+".bin");


    if(!bVocLoad)
    {
        cerr << "Cannot find binary file for vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile+".bin" << endl;
        cerr << "Trying to open the text file. This could take a while..." << endl;
        bool bVocLoad2 = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad2)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << strVocFile << endl;
            exit(-1);
        }
        cerr << "Saving the vocabulary to binary for the next time to " << strVocFile+".bin" << endl;
        mpVocabulary->saveToBinFile(strVocFile+".bin");
    }

    cout << "Vocabulary loaded!" << endl << endl;

    // begin map serialization addition
    // load serialized map
    if (load_map && LoadMap(map_file)) {
        std::cout << "Using loaded map with " << mpMap->MapPointsInMap() << " points\n" << std::endl;
    }
    else {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        //Create the Map
        mpMap = new Map();
    }
    // end map serialization addition


    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer,
                             mpMap, mpKeyFrameDatabase, mSensor, parameters);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&GPS_OFF_SLAM::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&GPS_OFF_SLAM::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    currently_localizing_only_ = false;


}

void System::TrackMonocular(const cv::Mat &im, const double &timestamp,const double &mLatitude, const double &mLongitude)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp,mLatitude,mLongitude);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    current_position_ = Tcw;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

bool System::isRunningGBA()
{
    return  mpLoopCloser->isRunningGBA();
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;


    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<GPS_OFF_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<GPS_OFF_SLAM::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        GPS_OFF_SLAM::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SetMinimumKeyFrames (int min_num_kf) {
  mpTracker->SetMinimumKeyFrames(min_num_kf);
}

cv::Mat System::GetCurrentPosition () {
  return current_position_;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

cv::Mat System::DrawCurrentFrame () {
  return mpFrameDrawer->DrawFrame();
}

std::vector<MapPoint*> System::GetAllMapPoints() {
  return mpMap->GetAllMapPoints();
}


bool System::SetCallStackSize (const rlim_t kNewStackSize) {
    struct rlimit rlimit;
    int operation_result;

    operation_result = getrlimit(RLIMIT_STACK, &rlimit);
    if (operation_result != 0) {
        std::cerr << "Error getting the call stack struct" << std::endl;
        return false;
    }

    if (kNewStackSize > rlimit.rlim_max) {
        std::cerr << "Requested call stack size too large" << std::endl;
        return false;
    }

    if (rlimit.rlim_cur <= kNewStackSize) {
        rlimit.rlim_cur = kNewStackSize;
        operation_result = setrlimit(RLIMIT_STACK, &rlimit);
        if (operation_result != 0) {
            std::cerr << "Setrlimit returned result: " << operation_result << std::endl;
            return false;
        }
        return true;
    }
    return false;
}


rlim_t System::GetCurrentCallStackSize () {
    struct rlimit rlimit;
    int operation_result;

    operation_result = getrlimit(RLIMIT_STACK, &rlimit);
    if (operation_result != 0) {
        std::cerr << "Error getting the call stack struct" << std::endl;
        return 16 * 1024L * 1024L; //default
    }

    return rlimit.rlim_cur;
}


void System::ActivateLocalizationMode()
{
    currently_localizing_only_ = true;
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    currently_localizing_only_ = false;
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::EnableLocalizationOnly (bool localize_only) {
  if (localize_only != currently_localizing_only_) {
    currently_localizing_only_ = localize_only;
    if (localize_only) {
      ActivateLocalizationMode();
    } else {
      DeactivateLocalizationMode();
    }
  }

  std::cout << "Enable localization only: " << (localize_only?"true":"false") << std::endl;
}


// map serialization addition
bool System::SaveMap(const string &filename) {
    unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
    std::ofstream out(filename, std::ios_base::binary);
    if (!out) {
        std::cerr << "cannot write to map file: " << map_file << std::endl;
        return false;
    }

    const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
    const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
    if (!SetCallStackSize(kNewStackSize)) {
        std::cerr << "Error changing the call stack size; Aborting" << std::endl;
        return false;
    }

    try {
        std::cout << "saving map file: " << map_file << std::flush;
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << mpMap;
        oa << mpKeyFrameDatabase;
        std::cout << " ... done" << std::endl;
        out.close();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        SetCallStackSize(kDefaultCallStackSize);
        return false;
    } catch (...) {
        std::cerr << "Unknows exeption" << std::endl;
        SetCallStackSize(kDefaultCallStackSize);
        return false;
    }

    SetCallStackSize(kDefaultCallStackSize);
    return true;
}

bool System::LoadMap(const string &filename) {

    unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot open map file: " << map_file << " , you need create it first!" << std::endl;
        return false;
    }

    const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
    const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
    if (!SetCallStackSize(kNewStackSize)) {
        std::cerr << "Error changing the call stack size; Aborting" << std::endl;
        return false;
    }

    std::cout << "Loading map file: " << map_file << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    std::cout << " ... done" << std::endl;

    std::cout << "Map reconstructing" << flush;
    vector<GPS_OFF_SLAM::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {

        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();

        if (it->mnFrameId > mnFrameId) {
            mnFrameId = it->mnFrameId;
        }
    }

    Frame::nNextId = mnFrameId;

    std::cout << " ... done" << std::endl;
    in.close();

    SetCallStackSize(kDefaultCallStackSize);

    return true;
}

} //namespace GPS_OFF_SLAM
