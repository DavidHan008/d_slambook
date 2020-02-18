#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "backend2.h"
#include "common_include2.h"
#include "dataset2.h"
#include "frontend2.h"
#include "viewer2.h"

namespace myslam {

/**
 * VO 对外接口
 */
class VisualOdometry {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    /// constructor with config file
    VisualOdometry();
    VisualOdometry(std::string &config_path);

    /**
     * do initialization things before run
     * @return true if success
     */
    bool Init();

    /**
     * start vo in the dataset
     */
    void Run();

    /**
     * Make a step forward in dataset
     */
    bool Step();

    /// 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

   private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};
}  // namespace myslam

#endif  // MYSLAM_VISUAL_ODOMETRY_H
