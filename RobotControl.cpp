#include <vector>
#include <fstream>
#include <iostream>
#include <QApplication>
#include "RobotControl.h" // Assumed header for TRTrans3D, TransMatrix3D, Point3D, etc.

void RobotControl::scRunSequence()
{
    // 保存左臂（ID=0）的初始姿态
    std::vector<TRTrans3D> initialPoses;
    getCurrentProbePoses(initialPoses);
    if (initialPoses.empty()) {
        std::cerr << "错误：无法获取初始姿态" << std::endl;
        return;
    }
    if (initialPoses.size() <= 0) {
        std::cerr << "错误：探头ID=0无可用姿态" << std::endl;
        return;
    }
    TRTrans3D initialPose = initialPoses[0]; // 左臂，ID=0

    // 将左臂沿y+方向移动200mm
    Point3D initialPosition;
    initialPose.getTranslation(initialPosition);
    double baseX = initialPosition.X();
    double baseY = initialPosition.Y();
    double baseZ = initialPosition.Z();
    TRTrans3D offsetPose = initialPose;
    offsetPose.setTranslation(baseX, baseY + 200.0, baseZ);
    setSingleProbeTarget(0, offsetPose);
    while (!isIdle()) {
        qApp->processEvents();
        if (_cancelSequence) {
            _cancelSequence = false;
            std::cerr << "警告：初始y轴偏移移动期间序列被取消" << std::endl;
            return;
        }
    }
    qApp->processEvents();

    // 从 Prediction_Point.txt 读取点
    std::vector<TRTrans3D> sequence;
    std::ifstream file("/Users/shichaozhang/USRobot/Prediction_Point.txt");
    if (!file.is_open()) {
        std::cerr << "错误：无法打开Prediction_Point.txt" << std::endl;
        return;
    }

    double matrix[4][4];
    int poseCount = 0;
    while (file.good()) {
        bool readSuccess = true;
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                if (!(file >> matrix[row][col])) {
                    readSuccess = false;
                    break;
                }
            }
            if (!readSuccess) break;
        }
        if (readSuccess) {
            poseCount++;
            // 跳过第一个姿态（原始姿态）
            if (poseCount == 1) {
                continue;
            }
            TransMatrix3D transMatrix;
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    transMatrix(row, col) = matrix[row][col];
                }
            }
            TRTrans3D pose(transMatrix);
            TRTrans3D alignedPose;
            if (!_scanSpace) {
                std::cerr << "警告：_scanSpace为空，无法对齐姿态" << std::endl;
                continue;
            }
            _scanSpace->getOffsetSurfaceAlignedPose(pose, alignedPose); // void返回，无成功检查
            sequence.push_back(alignedPose);
        }
    }
    file.close();

    // 检查是否读取到至少4个点
    if (sequence.size() < 4) {
        std::cerr << "错误：Prediction_Point.txt中的有效姿态不足4个，仅读取到 " << sequence.size() << " 个" << std::endl;
        return;
    }

    // 设置左臂为唯一活动探头
    clearActiveProbes();
    setActiveProbe(0, true);

    // 执行序列
    if (_cancelSequence) {
        _cancelSequence = false;
    }

    for (const auto& pose : sequence) {
        setSingleProbeTarget(0, pose);
        while (!isIdle()) {
            qApp->processEvents();
            if (_cancelSequence) {
                _cancelSequence = false;
                break;
            }
        }
        qApp->processEvents();

        if (_cancelSequence) {
            _cancelSequence = false;
            break;
        }
    }

    // 返回初始姿态
    setSingleProbeTarget(0, initialPose);
    while (!isIdle()) {
        qApp->processEvents();
    }
    qApp->processEvents();

    std::cerr << "scRunSequence完成，已返回初始姿态" << std::endl;
}
