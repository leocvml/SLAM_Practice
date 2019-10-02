/*


https://zhuanlan.zhihu.com/p/45404840
Some Rotation method

*/



#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "../Eigen/Core"
#include "../Eigen/Geometry"
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;
 
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef vector<Matrix<double, Dynamic, Dynamic>> TrajectoryType;
//typedef vector<Isometry3d>TrajectoryType;

void showPointCloud(
    const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);


Matrix<double,Dynamic,Dynamic> Quaterniond2RotationMatrix(double w,double x,double y,double z,double tx, double ty, double tz){
    Matrix<double,4,4> TransformMatrix;
    
    double lenth = sqrt((w*w + x*x +y*y +z*z));
    w = w/lenth;
    x = x/lenth;
    y = y/lenth;
    z = z/lenth;

    TransformMatrix(0,0) = 1 - 2*y*y -2*z*z;
    TransformMatrix(1,1) = 1 - 2*x*x -2*z*z;
    TransformMatrix(2,2) = 1 - 2*z*z -2*x*x;
    TransformMatrix(0,1) = 2*x*y - 2*z*w;
    TransformMatrix(0,2) = 2*x*z + 2*y*w;
    TransformMatrix(1,2) = 2*y*z - 2*x*w;
    TransformMatrix(1,0) = 2*x*y + 2*z*w;
    TransformMatrix(2,0) = 2*x*z - 2*y*w;;
    TransformMatrix(2,1) = 2*y*z + 2*x*w;
    TransformMatrix(0,3) = tx;
    TransformMatrix(1,3) = ty;
    TransformMatrix(2,3) = tz;
    TransformMatrix(3,3) = 1;
    TransformMatrix(3,0) = 0;
    TransformMatrix(3,1) = 0;
    TransformMatrix(3,2) = 0;
    
    return TransformMatrix;

}


Isometry3d ToTrajectory(double quater_w, double quater_x, double quater_y, double quater_z,double tx, double ty, double tz){
    Quaterniond q1(quater_w, quater_x, quater_y, quater_z);
    //cout << q1.coeffs().transpose()<<endl;
    q1.normalize();
    //cout << q1.coeffs().transpose()<<endl;
    Vector3d t1(tx,ty,tz);
    Isometry3d Trans(q1);
    Trans.pretranslate(t1);

    return Trans;
}



int main(){


 
 

    
    vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryType pose;
    ifstream fin("../pose.txt");
    if(!fin){
        cout <<"could not found file"<<endl;
    }

    stringstream Strstm;
    string Colorpath = "../color/";
    string Depthpath = "../depth/";
    string strTemp;
    string colorImgName;
    for (int i = 1; i <= 5; i++){
        Strstm.clear();
        strTemp.clear();
        Strstm << i ;
        Strstm >> strTemp;
     

   
        colorImgs.push_back(cv::imread((Colorpath + strTemp+".png"),CV_LOAD_IMAGE_COLOR));
        depthImgs.push_back(cv::imread(Depthpath + strTemp+".pgm",-1));

        double data[7] = {0};
        double a;
        for(auto &d:data)
            fin>>d;
        //Isometry3d TransMatrix = ToTrajectory(data[6],data[3], data[4],data[5],data[0],data[1],data[2]);
        Matrix<double,Dynamic,Dynamic>  Trans = Quaterniond2RotationMatrix(data[6],data[3], data[4],data[5],data[0],data[1],data[2]);
    
        pose.push_back(Trans);
    
    }
    
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d> > pointcloud;
    pointcloud.reserve(1000000);

    for(int i =0; i < 5; i++){
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Matrix<double,Dynamic,Dynamic> T = pose[i];
        //Isometry3d T = pose[i];
        
        for(int v=0; v<color.rows;v++){
            for(int u=0; u<color.cols; u++){
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                if (d==0) continue;
        
                //Eigen::Vector3d point;
                Eigen::Matrix<double,4,1> point;
                point[2] = double(d)/ depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                point[3] = 1;

                Eigen::Matrix<double,4,1>pointWorld = T * point;
                //Eigen::Vector3d pointWorld = T * point;  
               

                Vector6d p;
                p.head<3>() = pointWorld.head<3>();
                //p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];
                p[4] = color.data[v * color.step + u * color.channels() + 1];
                p[3] = color.data[v * color.step + u * color.channels() + 2];
                pointcloud.push_back(p);
            }
        }
    }

    cout << pointcloud.size() <<endl;
    showPointCloud(pointcloud);



    return 0;
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        //usleep(5000);   // sleep 5 ms
    }
    return;
}
