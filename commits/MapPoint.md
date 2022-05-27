

MapPoint在tracking的UpdateLastFrame里被创建出来，是通过相机模型把2D的像素坐标转换成3D的MapPoint的

MapPoint的重要成员
    // Reference KeyFrame
    KeyFrame* mpRefKF;

    // Best descriptor to fast matching
    cv::Mat mDescriptor; // 一个地图点可以被许多关键帧看到，每一个都有一个描述子，取位置处于中间的那个描述子作为该地图点的描述子