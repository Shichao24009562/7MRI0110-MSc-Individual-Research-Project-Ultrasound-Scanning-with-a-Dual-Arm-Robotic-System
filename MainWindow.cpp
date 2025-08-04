#include <QKeyEvent>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QMainWindow>
#include <cmath>
#include "RobotControl.h" // Assumed header for TRTrans3D, Point3D, etc.

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        switch (event->key()) {
        case Qt::Key_P:
        {
            // 进入手动模式并暂停
            _robotControl->setManualMode(true);
            _robotControl->setContinueRun(false);
            qDebug() << "[手动模式] 当前步骤暂停";

            // 捕获右臂（ID=1）姿态
            std::vector<TRTrans3D> currentPoses;
            _robotControl->getCurrentProbePoses(currentPoses);
            if (currentPoses.size() <= 1) {
                qDebug() << "[错误] 无法获取当前姿态或探头ID=1无效";
                break;
            }
            TRTrans3D currentPose = currentPoses[1]; // 右臂，ID=1

            // 绕z轴旋转180度
            TransMatrix3D rotationMatrix;
            double angle = M_PI; // 180度（弧度）
            rotationMatrix(0, 0) = cos(angle);
            rotationMatrix(0, 1) = -sin(angle);
            rotationMatrix(0, 2) = 0;
            rotationMatrix(0, 3) = 0;
            rotationMatrix(1, 0) = sin(angle);
            rotationMatrix(1, 1) = cos(angle);
            rotationMatrix(1, 2) = 0;
            rotationMatrix(1, 3) = 0;
            rotationMatrix(2, 0) = 0;
            rotationMatrix(2, 1) = 0;
            rotationMatrix(2, 2) = 1;
            rotationMatrix(2, 3) = 0;
            rotationMatrix(3, 0) = 0;
            rotationMatrix(3, 1) = 0;
            rotationMatrix(3, 2) = 0;
            rotationMatrix(3, 3) = 1;

            TRTrans3D rotatedPose = currentPose * TRTrans3D(rotationMatrix);

            // 保存到 p_point.txt
            QFile pPointFile("/Users/shichaozhang/USRobot/p_point.txt");
            if (!pPointFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                qDebug() << "[错误] 无法打开 p_point.txt 进行写入: " << pPointFile.errorString();
                break;
            }
            QTextStream out(&pPointFile);
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    out << QString::number(rotatedPose(i, j), 'f', 5) << " ";
                }
                out << "\n";
            }
            pPointFile.close();
            qDebug() << "[成功] 旋转后的姿态已保存到 p_point.txt";

            // 生成并保存4个额外点到 Prediction_Point.txt
            QFile predFile("/Users/shichaozhang/USRobot/Prediction_Point.txt");
            if (!predFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                qDebug() << "[错误] 无法打开 Prediction_Point.txt 进行写入: " << predFile.errorString();
                break;
            }
            QTextStream predOut(&predFile);

            // 写入旋转后的姿态
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    predOut << QString::number(rotatedPose(i, j), 'f', 5) << " ";
                }
                predOut << "\n";
            }

            // 生成并写入4个新点，y坐标递增5mm
            Point3D basePosition;
            rotatedPose.getTranslation(basePosition);
            double baseX = basePosition.X();
            double baseY = basePosition.Y();
            double baseZ = basePosition.Z();
            for (int k = 1; k <= 4; ++k) {
                double newY = baseY + 100 + (k-1) * 5.0; // y 初始偏移100mm，每次递增5mm
                TRTrans3D newPose = rotatedPose;
                newPose.setTranslation(baseX, newY, baseZ);
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        predOut << QString::number(newPose(i, j), 'f', 5) << " ";
                    }
                    predOut << "\n";
                }
            }
            predFile.close();
            qDebug() << "[成功] 4个点已保存到 Prediction_Point.txt";
            break;
        }
        case Qt::Key_S:
        {
            qDebug() << "[扫描模式] 使用左臂（ID=0）开始扫描";
            _robotControl->scRunSequence();
            break;
        }
        case Qt::Key_Left:
        {
            _robotControl->requestStepBack();
            qDebug() << "[手动模式] 请求后退一步";
            break;
        }
        case Qt::Key_Right:
        {
            _robotControl->requestStepForward();
            qDebug() << "[手动模式] 请求前进一步";
            break;
        }
        case Qt::Key_C:
        {
            _robotControl->setContinueRun(true);
            qDebug() << "[手动模式] 继续完成序列";
            break;
        }
        default:
            break;
        }
    }

    QMainWindow::keyPressEvent(event);
}
