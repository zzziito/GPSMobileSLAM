#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <sys/resource.h>
#include <fstream>


#include "Tracking.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

namespace GPS_OFF_SLAM
{
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

struct ORBParameters;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string strVocFile, const eSensor sensor, ORBParameters& parameters,
           const std::string & map_file = "", bool load_map = false); // map serialization addition

    // Process the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    void TrackMonocular(const cv::Mat &im, const double &timestamp,const double &mLatitude, const double &mLongitude);
    
    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Returns true if Global Bundle Adjustment is running
    bool isRunningGBA();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    void SaveTrajectoryKITTI(const string &filename);

    //Checks the current mode (mapping or localization) and changes the mode if requested
    void EnableLocalizationOnly (bool localize_only);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    void SetMinimumKeyFrames (int min_num_kf);

    bool SaveMap(const string &filename);

    cv::Mat GetCurrentPosition ();

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    cv::Mat DrawCurrentFrame ();

    std::vector<MapPoint*> GetAllMapPoints();

private:
    bool SetCallStackSize (const rlim_t kNewStackSize);

    rlim_t GetCurrentCallStackSize ();

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();

    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    bool LoadMap(const string &filename);

    bool currently_localizing_only_;

    bool load_map;

    std::string map_file;

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    FrameDrawer* mpFrameDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    // Current position
    cv::Mat current_position_;

};

}// namespace GPS_OFF_SLAM

#endif // SYSTEM_H
