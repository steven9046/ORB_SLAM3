KeyFrame主要由tracking线程创建，然后localMapping线程进行维护
KeyFrame包含Frame里的大部分功能(按理来说应该是Frame的派生类，但是他这里没有这么设计)

KeyFrame中重要成员
    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    Map* mpMap; // 从哪个map创建的(KeyFrame构造需要传入一个Map)
    unsigned int mnOriginMapId; // Map的编号

与共视关系有关的成员 
    //     共视关键帧   共视点数量
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    // 按照共视程度排序
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // spanning tree
    // 关键帧之间是有这种树状关系的
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;