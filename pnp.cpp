#include "glog/logging.h"

#include "opencv2/core/core.hpp"
#include <vector>

#include "ceres/ceres.h"
#include "ceres/rotation.h"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace cv;
using namespace std;


// Templated pinhole camera projection model with unkown extrinsics.
// The camera is precalibrated with known intrinsics.
// Images are assumed to have been undistorted. Correspondence between scene an image points
// are known.
// The extrinsics are parameterized using 6 parameters: 3 for rotation, 3 for translation.
struct ReprojectionError {
  ReprojectionError(double fx,double fy, double cx, double cy,
                    const cv::Point2f& image_point,
                    const cv::Point3f& scene_point)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        image_point_(image_point),
        scene_point_(scene_point){}

  template <typename T>
  bool operator()(const T* const extrinsic,
                  T* residuals) const {

      // camera[0,1,2] are the angle-axis rotation.
      T sp[3];
      sp[0] = T(scene_point_.x);
      sp[1] = T(scene_point_.y);
      sp[2] = T(scene_point_.z);

      T p[3];
      ceres::AngleAxisRotatePoint(extrinsic, sp, p);

      // camera[3,4,5] are the translation.
      p[0] += extrinsic[3];
      p[1] += extrinsic[4];
      p[2] += extrinsic[5];

      T xp, yp;
      //now project the point to the camera space
      if(fabs(scene_point_.z) < 0.01)
      {
          xp = T(fx_) * sp[0]  + T(cx_);
          yp = T(fy_) * sp[1]  + T(cy_);
      }
      else
      {
          xp = T(fx_) * sp[0] / sp[2] + T(cx_);
          yp = T(fy_) * sp[1] / sp[2] + T(cy_);
      }


    // The error is the difference between the predicted and observed position.
    residuals[0] = T(image_point_.x) - xp;
    residuals[1] = T(image_point_.y) - yp;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double fx,double fy, double cx, double cy,
                                     const cv::Point2f& image_point,
                                     const cv::Point3f& scene_point) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
                new ReprojectionError(fx, fy, cx, cy, image_point, scene_point)));
  }

  double fx_, fy_, cx_, cy_; //calibrated camera params. No distortion.
  cv::Point2f image_point_;  // point on image
  cv::Point3f scene_point_;  // and corresponding point in scene

};



void pnpTest()
{
    vector<Point2f> corners2D;
    vector<Point3f> corners3D;
    {
    corners2D.push_back(cv::Point2f(433.576, 767.778));    corners2D.push_back(cv::Point2f(532.603, 770.827));
    corners2D.push_back(cv::Point2f(630.78, 773.785));    corners2D.push_back(cv::Point2f(727.762, 776.714));
    corners2D.push_back(cv::Point2f(824.016, 779.73));    corners2D.push_back(cv::Point2f(919.323, 782.843));
    corners2D.push_back(cv::Point2f(1013.63, 785.665));    corners2D.push_back(cv::Point2f(1106.9, 788.783));
    corners2D.push_back(cv::Point2f(1199.34, 791.62));    corners2D.push_back(cv::Point2f(1290.87, 794.697));
    corners2D.push_back(cv::Point2f(430.489, 863.934));    corners2D.push_back(cv::Point2f(529.854, 866.56));
    corners2D.push_back(cv::Point2f(628.315, 869.003));    corners2D.push_back(cv::Point2f(725.804, 871.607));
    corners2D.push_back(cv::Point2f(822.409, 874.23));    corners2D.push_back(cv::Point2f(918.118, 876.786));
    corners2D.push_back(cv::Point2f(1012.63, 879.352));    corners2D.push_back(cv::Point2f(1106.35, 881.731));
    corners2D.push_back(cv::Point2f(1198.97, 884.319));    corners2D.push_back(cv::Point2f(1290.93, 886.91));
    corners2D.push_back(cv::Point2f(427.209, 960.991));    corners2D.push_back(cv::Point2f(527.161, 963.132));
    corners2D.push_back(cv::Point2f(626.073, 965.228));    corners2D.push_back(cv::Point2f(723.812, 967.315));
    corners2D.push_back(cv::Point2f(820.745, 969.419));    corners2D.push_back(cv::Point2f(916.663, 971.414));
    corners2D.push_back(cv::Point2f(1011.73, 973.503));    corners2D.push_back(cv::Point2f(1105.73, 975.54));
    corners2D.push_back(cv::Point2f(1198.72, 977.626));    corners2D.push_back(cv::Point2f(1290.86, 979.701));
    corners2D.push_back(cv::Point2f(424.326, 1058.69));    corners2D.push_back(cv::Point2f(524.335, 1060.34));
    corners2D.push_back(cv::Point2f(623.685, 1061.91));    corners2D.push_back(cv::Point2f(721.818, 1063.56));
    corners2D.push_back(cv::Point2f(819.111, 1065.24));    corners2D.push_back(cv::Point2f(915.415, 1066.82));
    corners2D.push_back(cv::Point2f(1010.65, 1068.29));    corners2D.push_back(cv::Point2f(1105.01, 1069.91));
    corners2D.push_back(cv::Point2f(1198.45, 1071.58));    corners2D.push_back(cv::Point2f(1290.88, 1073.31));
    corners2D.push_back(cv::Point2f(420.947, 1157.26));    corners2D.push_back(cv::Point2f(521.642, 1158.37));
    corners2D.push_back(cv::Point2f(621.343, 1159.42));    corners2D.push_back(cv::Point2f(719.799, 1160.48));
    corners2D.push_back(cv::Point2f(817.534, 1161.65));    corners2D.push_back(cv::Point2f(914.233, 1162.85));
    corners2D.push_back(cv::Point2f(1009.83, 1163.83));    corners2D.push_back(cv::Point2f(1104.41, 1165.16));
    corners2D.push_back(cv::Point2f(1198.21, 1166.16));    corners2D.push_back(cv::Point2f(1290.86, 1167.47));
    corners2D.push_back(cv::Point2f(417.789, 1256.41));    corners2D.push_back(cv::Point2f(518.833, 1256.89));
    corners2D.push_back(cv::Point2f(618.905, 1257.52));    corners2D.push_back(cv::Point2f(717.771, 1258.32));
    corners2D.push_back(cv::Point2f(815.774, 1258.76));    corners2D.push_back(cv::Point2f(912.707, 1259.62));
    corners2D.push_back(cv::Point2f(1008.7, 1260.1));    corners2D.push_back(cv::Point2f(1103.76, 1260.77));
    corners2D.push_back(cv::Point2f(1197.74, 1261.53));    corners2D.push_back(cv::Point2f(1290.76, 1262.28));
    corners2D.push_back(cv::Point2f(414.545, 1356.24));    corners2D.push_back(cv::Point2f(515.921, 1356.36));
    corners2D.push_back(cv::Point2f(616.407, 1356.6));    corners2D.push_back(cv::Point2f(715.772, 1356.56));
    corners2D.push_back(cv::Point2f(813.993, 1356.84));    corners2D.push_back(cv::Point2f(911.481, 1356.97));
    corners2D.push_back(cv::Point2f(1007.8, 1357.07));    corners2D.push_back(cv::Point2f(1103.08, 1357.21));
    corners2D.push_back(cv::Point2f(1197.47, 1357.47));    corners2D.push_back(cv::Point2f(1290.73, 1357.68));
    corners2D.push_back(cv::Point2f(411.344, 1456.84));    corners2D.push_back(cv::Point2f(513.051, 1456.61));
    corners2D.push_back(cv::Point2f(613.763, 1456.17));    corners2D.push_back(cv::Point2f(713.526, 1455.89));
    corners2D.push_back(cv::Point2f(812.401, 1455.43));    corners2D.push_back(cv::Point2f(909.952, 1455.06));
    corners2D.push_back(cv::Point2f(1006.7, 1454.69));    corners2D.push_back(cv::Point2f(1102.51, 1454.52));
    corners2D.push_back(cv::Point2f(1197.25, 1454.23));    corners2D.push_back(cv::Point2f(1290.86, 1453.83));

    corners3D.push_back(cv::Point3f(0, 266.7, 100));    corners3D.push_back(cv::Point3f(38.1, 266.7, 100));
    corners3D.push_back(cv::Point3f(76.2, 266.7, 100));    corners3D.push_back(cv::Point3f(114.3, 266.7, 100));
    corners3D.push_back(cv::Point3f(152.4, 266.7, 100));    corners3D.push_back(cv::Point3f(190.5, 266.7, 100));
    corners3D.push_back(cv::Point3f(228.6, 266.7, 100));    corners3D.push_back(cv::Point3f(266.7, 266.7, 100));
    corners3D.push_back(cv::Point3f(304.8, 266.7, 100));    corners3D.push_back(cv::Point3f(342.9, 266.7, 100));
    corners3D.push_back(cv::Point3f(0, 228.6, 100));    corners3D.push_back(cv::Point3f(38.1, 228.6, 100));
    corners3D.push_back(cv::Point3f(76.2, 228.6, 100));    corners3D.push_back(cv::Point3f(114.3, 228.6, 100));
    corners3D.push_back(cv::Point3f(152.4, 228.6, 100));    corners3D.push_back(cv::Point3f(190.5, 228.6, 100));
    corners3D.push_back(cv::Point3f(228.6, 228.6, 100));    corners3D.push_back(cv::Point3f(266.7, 228.6, 100));
    corners3D.push_back(cv::Point3f(304.8, 228.6, 100));    corners3D.push_back(cv::Point3f(342.9, 228.6, 100));
    corners3D.push_back(cv::Point3f(0, 190.5, 100));    corners3D.push_back(cv::Point3f(38.1, 190.5, 100));
    corners3D.push_back(cv::Point3f(76.2, 190.5, 100));    corners3D.push_back(cv::Point3f(114.3, 190.5, 100));
    corners3D.push_back(cv::Point3f(152.4, 190.5, 100));    corners3D.push_back(cv::Point3f(190.5, 190.5, 100));
    corners3D.push_back(cv::Point3f(228.6, 190.5, 100));    corners3D.push_back(cv::Point3f(266.7, 190.5, 100));
    corners3D.push_back(cv::Point3f(304.8, 190.5, 100));    corners3D.push_back(cv::Point3f(342.9, 190.5, 100));
    corners3D.push_back(cv::Point3f(0, 152.4, 100));    corners3D.push_back(cv::Point3f(38.1, 152.4, 100));
    corners3D.push_back(cv::Point3f(76.2, 152.4, 100));    corners3D.push_back(cv::Point3f(114.3, 152.4, 100));
    corners3D.push_back(cv::Point3f(152.4, 152.4, 100));    corners3D.push_back(cv::Point3f(190.5, 152.4, 100));
    corners3D.push_back(cv::Point3f(228.6, 152.4, 100));    corners3D.push_back(cv::Point3f(266.7, 152.4, 100));
    corners3D.push_back(cv::Point3f(304.8, 152.4, 100));    corners3D.push_back(cv::Point3f(342.9, 152.4, 100));
    corners3D.push_back(cv::Point3f(0, 114.3, 100));    corners3D.push_back(cv::Point3f(38.1, 114.3, 100));
    corners3D.push_back(cv::Point3f(76.2, 114.3, 100));    corners3D.push_back(cv::Point3f(114.3, 114.3, 100));
    corners3D.push_back(cv::Point3f(152.4, 114.3, 100));    corners3D.push_back(cv::Point3f(190.5, 114.3, 100));
    corners3D.push_back(cv::Point3f(228.6, 114.3, 100));    corners3D.push_back(cv::Point3f(266.7, 114.3, 100));
    corners3D.push_back(cv::Point3f(304.8, 114.3, 100));    corners3D.push_back(cv::Point3f(342.9, 114.3, 100));
    corners3D.push_back(cv::Point3f(0, 76.2, 100));    corners3D.push_back(cv::Point3f(38.1, 76.2, 100));
    corners3D.push_back(cv::Point3f(76.2, 76.2, 100));    corners3D.push_back(cv::Point3f(114.3, 76.2, 100));
    corners3D.push_back(cv::Point3f(152.4, 76.2, 100));    corners3D.push_back(cv::Point3f(190.5, 76.2, 100));
    corners3D.push_back(cv::Point3f(228.6, 76.2, 100));    corners3D.push_back(cv::Point3f(266.7, 76.2, 100));
    corners3D.push_back(cv::Point3f(304.8, 76.2, 100));    corners3D.push_back(cv::Point3f(342.9, 76.2, 100));
    corners3D.push_back(cv::Point3f(0, 38.1, 100));    corners3D.push_back(cv::Point3f(38.1, 38.1, 100));
    corners3D.push_back(cv::Point3f(76.2, 38.1, 100));    corners3D.push_back(cv::Point3f(114.3, 38.1, 100));
    corners3D.push_back(cv::Point3f(152.4, 38.1, 100));    corners3D.push_back(cv::Point3f(190.5, 38.1, 100));
    corners3D.push_back(cv::Point3f(228.6, 38.1, 100));    corners3D.push_back(cv::Point3f(266.7, 38.1, 100));
    corners3D.push_back(cv::Point3f(304.8, 38.1, 100));    corners3D.push_back(cv::Point3f(342.9, 38.1, 100));
    corners3D.push_back(cv::Point3f(0, 0, 100));    corners3D.push_back(cv::Point3f(38.1, 0, 100));
    corners3D.push_back(cv::Point3f(76.2, 0, 100));    corners3D.push_back(cv::Point3f(114.3, 0, 100));
    corners3D.push_back(cv::Point3f(152.4, 0, 100));    corners3D.push_back(cv::Point3f(190.5, 0, 100));
    corners3D.push_back(cv::Point3f(228.6, 0, 100));    corners3D.push_back(cv::Point3f(266.7, 0, 100));
    corners3D.push_back(cv::Point3f(304.8, 0, 100));    corners3D.push_back(cv::Point3f(342.9, 0, 100));
    }


    ////camera params
    double fx = 2.3110603987479462e+03;
    double fy = fx;
    double cx = 1.0248040011791225e+03;
    double cy = 1.0094513637508215e+03;

    ////ground truth
    //Mat K = Mat::eye(3,3,CV_64FC1);
    //K.at<double>(0,0) = fx;    K.at<double>(1,1) = fy;
    //K.at<double>(0,2) = cx;    K.at<double>(1,2) = cy;
    //Mat kc = Mat::zeros(8,1,CV_64FC1);
    //Mat rvec, tvec;
    //cv::solvePnPRansac(corners3D, corners2D, K, kc, rvec, tvec);
    //double *R = (double*)rvec.data;
    //double *T = (double*)tvec.data;
    //double gt[] = {R[0], R[1], R[2], T[0], T[1], T[2]};
    double gt[] = {3.04985, 0.0223962, 0.184204, -246.12, 179.33, 981.048};
    //initialize the params to something close to the gt
    double ext[] = {gt[0], gt[1], gt[2], gt[3]+1000, gt[4]+1000, gt[5]+1000};


    cout << endl << "Ground truth: ";
    cout << gt[0] << ", " << gt[1] <<", " << gt[2] << ", ";
    cout << gt[3] << ", " << gt[4] <<", " << gt[5] << endl;
    cout << endl << "Inital pose: ";
    cout << ext[0] << ", " << ext[1] <<", " << ext[2] << ", ";
    cout << ext[3] << ", " << ext[4] <<", " << ext[5] << endl;

    ceres::Problem problem;

    for (int i = 0; i < corners2D.size(); ++i) {

      ceres::CostFunction* cost_function =
              ReprojectionError::Create(fx, fy,cx, cy,
                                        corners2D[i], corners3D[i]);
      problem.AddResidualBlock(cost_function,
                               NULL /* squared loss */,
                               ext);
    }

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";


    cout << endl << "Final pose: ";
    cout << ext[0] << ", " << ext[1] <<", " << ext[2] << ", ";
    cout << ext[3] << ", " << ext[4] <<", " << ext[5] << endl;

}

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);

    pnpTest();
    return 0;
}
