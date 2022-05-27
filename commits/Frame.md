tracking 里有两个 Frame 实例
    mCurrentFrame  
    mLastFrame

通过 GrabImageRGBD 函数来构造当前帧

Frame 中的几个重要成员
    Sophus::SE3<float> mTcw; // 这一帧的位姿

    ORBextractor* mpORBextractorLeft; // ORB特征提取器

    int N; // 特征点数量

    std::vector<cv::KeyPoint> mvKeys; // 特征点像素坐标

    std::vector<cv::KeyPoint> mvKeysUn; // 去畸变后的特征点像素坐标

    std::vector<MapPoint*> mvpMapPoints; // 3D 地图点

    cv::Mat mDescriptors; // 所有特征点对应的ORB描述子
    
    // 经过八叉树把特征点分配到格子里
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    Frame* mpPrevFrame; // 前一帧指针

    mnId; // Frame ID

    KeyFrame* mpReferenceKF; // 参考关键帧

    // std::map<WordId, WordValue>  
    // 词典里训练出来的 <词汇id, 词汇值>
    // 同一个词汇出现多次时 value 相加
    DBoW2::BowVector mBowVec; 

    // 特征向量,通过 Frame::ComputeBoW() 计算出
    // std::map<NodeId, std::vector<unsigned int> >  
    // 每一个描述子对应的 <词汇树里位置(节点号)，<描述子向量里位置(第几个描述子)>>
    // 同一个 node id 出现多次时，描述子的id都存到vector里
    DBoW2::FeatureVector mFeatVec; 

