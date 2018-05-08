#include <visp/vpImage.h> 
#include <visp_ros/vpROSGrabber.h>
#include <iostream>
#include "ros/ros.h" 
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpKeyboard.h>
#include <stdio.h>
#include <visp3/core/vpColor.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <std_msgs/Int8.h>
#include "handeye.h"
#include "kinova_arm_moveit_demo/toolposeChange.h"

using namespace std;
using namespace cv;

int positionSignal=-1;    //到达指定位姿后标志位改变
vector<Mat> Hgij;         //AX=XB中的B
vector<Mat> Hbt;         //基坐标系下机械臂末端位姿
vector<Mat> eye2hand;	//相机坐标系下机械臂末端位姿

//接收track_points发布的标志位
void handeye_signal_subCB(const std_msgs::Int8::ConstPtr &signal_sub)
{
  
  positionSignal=(int)signal_sub->data;
  cout << "positionSignal is " << positionSignal << endl;
}

//接收机械臂位姿变化矩阵
void pose_change_subCB(const kinova_arm_moveit_demo::toolposeChange::ConstPtr &poseChange)
{
  Mat b1(4, 4, CV_64FC1, Scalar::all(0));
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
    {
	  b1.at<double>(i,j)=poseChange->pose_change[4*i+j];
	}		
  }
  Hgij.push_back(b1.clone());
  cout << "poseChange is " << b1 << endl;
}

//接收订阅机械臂末端位姿矩阵
void pose_subCB(const kinova_arm_moveit_demo::toolposeChange::ConstPtr &pose)
{
  Mat b1(4, 4, CV_64FC1, Scalar::all(0));
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)	
    {
	  b1.at<double>(i,j)=pose->pose_change[4*i+j];
	}		
  }
  Hbt.push_back(b1.clone());
  cout << "pose is " << b1 << endl;
}

//旋转矩阵和平移向量转齐次矩阵
Mat rt2m(const Mat &pose_t,const Mat &pose_r)
{
	Mat pose_m = Mat::eye(cv::Size(4,4),CV_64FC1); 
	for(int j=0; j<3; j++)
  	{
    	for(int k=0; k<3; k++)	
		{
	 	 	pose_m.at<double>(j,k)=pose_r.at<double>(j,k);
		}		
  	}
	for(int j=0; j<3; j++)
	{
	  	pose_m.at<double>(j,3)=pose_t.at<double>(j,0)/1000;
	}
	//pose_m(3,3)=1.0;
	cout<<"齐次矩阵: "<<endl<<pose_m<<endl;
	return pose_m;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "hand_eye_calculate");   //定义节点名称
	ros::NodeHandle handle;   //为这个进程的节点创建一个句柄
	//发布话题

	//接收话题
	ros::Subscriber signal_sub=handle.subscribe("hand_eye_arm_signal", 1, handeye_signal_subCB); //订阅机械臂运动标志位
	ros::Subscriber pose_change_sub=handle.subscribe("hand_eye_arm_pose_change", 10, pose_change_subCB); //订阅机械臂位姿变化矩阵
	ros::Subscriber pose_sub=handle.subscribe("hand_eye_arm_pose", 10, pose_subCB); //订阅机械臂末端位姿矩阵

	vpImage<unsigned char> I;  //visp用于存储捕捉到的最新图像
	Mat image;             //OpenCV存储捕捉到的最新图像
	vpROSGrabber g;
	g.setImageTopic("/kinect2/hd/image_color_rect");
	g.open(I);          //打开相机
	g.acquire(I);       //获取图像
	vpImageConvert::convert(I, image);   //图像类型转换
	
	
	//参数变量
	const int image_count = 5;	               //需要保存的图片数量
	int count = 0;  //计算保存的标定图像数量
	int n = 0;        //保存图片的名称
	stringstream tempname;
	string filename;
	Size image_size = image.size();     //图像的尺寸
	Size board_size = Size(11, 8);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	string msg;
	int baseLine;
	Size textSize;
	int key;         //记录按键值

	//solvePnP参数
	vector<Point3f> objP;
	objP.push_back(Point3f(0,0,0));
	objP.push_back(Point3f(0,150,0));		//(0,150,0)
	objP.push_back(Point3f(0,300,0));
	objP.push_back(Point3f(210,0,0));
	objP.push_back(Point3f(210,150,0));
	objP.push_back(Point3f(210,300,0));
	Mat objPM;
	Mat(objP).convertTo(objPM,CV_32F);
	//相机参数
	double camD[9] = {1.0666158844675942e+03, 0, 9.6491286368427427e+02,
  0, 1.0665243381829043e+03, 5.5196014909449934e+02,
  0, 0, 1};
	double distCoeffD[5] = {8.1787249718780958e-02, -1.5234285275636286e-01,
       -8.5755198488431949e-07, 5.0012874507982625e-04,
       7.0182328076515657e-02};
	Mat camera_matrix = Mat(3,3,CV_64FC1,camD);
	Mat distortion_coefficients = Mat(5,1,CV_64FC1,distCoeffD);

	//计算手眼关系参数
    vector<Mat> v_rotM;
	vector<Mat> v_tvec;

	cv::namedWindow("camera exterior calibration", cv::WINDOW_AUTOSIZE);
	while (n < image_count && ros::ok())
	{
		g.acquire(I);       //获取图像
		vpImageConvert::convert(I, image);   //图像类型转换   
		if (image.empty())
		{
			cout << "img is empty" << endl;
			g.close();
			return -1;
		}

		//Mat imageGray = image;			
		//cvtColor(image, imageGray, CV_RGB2GRAY);	//彩色图片需要转化为灰度图

		// 提取角点
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		key = 0xff & waitKey(50);
		ros::spinOnce();
		if ((key & 255) == 32  || positionSignal!=-1)   //  空格键
		{
			positionSignal=-1;   //重新赋值，为下一个位姿做准备
			if (patternfound)
			{
				n++;
				tempname << "/home/qcrong/Documents/calibration_result/eye_to_hand/" << n;
				tempname >> filename;
				filename += ".jpg";
				/* 亚像素精确化 */
				cornerSubPix(image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				count += corners.size();
				imwrite(filename, image);
				tempname.clear();
				filename.clear();

				vector<Point2f> imgP;
				imgP.push_back(corners.at(0));
				imgP.push_back(corners.at(5));
				imgP.push_back(corners.at(10));
				imgP.push_back(corners.at(77));
				imgP.push_back(corners.at(82));
				imgP.push_back(corners.at(87));
				
				//solvePnP求解相机坐标系下目标物的位姿
				vector<double> rv(3), tv(3);
				Mat rvec(rv), tvec(tv);
				double rm[9];
				Mat rotM(3,3,CV_64FC1,rm);

				Rodrigues(rotM,rvec);
				solvePnP(objPM, Mat(imgP), camera_matrix, distortion_coefficients, rvec, tvec);
				Rodrigues(rvec,rotM);

				v_rotM.push_back(rotM.clone());
  				v_tvec.push_back(tvec.clone());
				eye2hand.push_back(rt2m(tvec,rotM));
				cout<<"图片数量：  "<<n<<endl;
				cout<<"rotation matrix: "<<endl<<rotM<<endl;  
    				//cout<<"translation matrix: "<<endl<<tv[0]<<" "<<tv[1]<<" "<<tv[2]<<endl;
				cout<<"tvec matrix: "<<endl<<tvec.at<double>(0,0)<<" "<<tvec.at<double>(1,0)<<" "<<tvec.at<double>(2,0)<<endl;
				//cout<<"齐次矩阵: "<<endl<<eye2hand[]<<endl;
			}
			else
			{
				std::cout << "Detect Failed.\n";
			}
		}
		else if ((key & 255) == 27)     //按esc键退出
		{
			break;
		}

		for(int i=0;i<corners.size();i++)
		{
			cv::circle(image,corners.at(i),2,CV_RGB(0,0,255),1);
		}

		baseLine = 0;
		textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(image.cols - 2 * textSize.width - 10, image.rows - 2 * baseLine - 10);
		msg = format("Press 'esc' to quit  %d/%d", n, image_count);
		putText(image, msg, textOrigin, 1, 1, CV_RGB(0, 0, 255));
		cv::imshow("camera exterior calibration", image);
	}
	g.close();

	if(n==image_count)  //达到计数要求，开始标定手眼关系
	{
		ofstream hand_eye_fout("/home/qcrong/Documents/calibration_result/eye_to_hand/hand_eye_result.txt");  /**    保存定标结果的文件     **/
		vector<Mat> Hcij;		

		for(int i=1;i<image_count;i++)
		{
			Mat a1(4, 4, CV_64FC1, Scalar::all(0));
			Mat a_r=Mat(3,3,CV_64FC1,Scalar::all(0)); 
			Mat a_t=Mat(3,1,CV_64FC1,Scalar::all(0));
			//a_r=v_rotM.at(i-1)*v_rotM.at(i).t();
			//a_t=v_rotM.at(i-1)*(-v_rotM.at(i).t()*v_tvec.at(i))+v_tvec.at(i-1);
			a_r=v_rotM.at(i-1).t()*v_rotM.at(i);
			a_t=v_rotM.at(i-1).t()*v_tvec.at(i)-v_rotM.at(i-1).t()*v_tvec.at(i-1);
			
			for(int j=0; j<3; j++)
			{
				for(int k=0; k<3; k++)	
				{
					a1.at<double>(j,k)=a_r.at<double>(j,k);
				}		
			}
			for(int j=0; j<3; j++)
			{
				a1.at<double>(j,3)=a_t.at<double>(j,0)/1000;
			}
			a1.at<double>(3,3)=1.0;

			Hcij.push_back(a1.clone());
		}
		
		for(int i=0;i<Hcij.size();i++)
		{
			cout<<"Hcij "<< i << endl <<Hcij.at(i)<<endl;
		}
		for(int i=0;i<Hgij.size();i++)
		{
			cout<<"Hgij "<< i << endl <<Hgij.at(i)<<endl;
		}

		Mat Hcg1(4, 4, CV_64FC1);
		Tsai_HandEye(Hcg1, Hgij, Hcij);

		Mat Hcg2(4, 4, CV_64FC1);
		DualQ_HandEye(Hcg2, Hgij, Hcij);

		Mat Hcg3(4, 4, CV_64FC1);
		Inria_HandEye(Hcg3, Hgij, Hcij);

		Mat Hcg4(4, 4, CV_64FC1);
		Navy_HandEye(Hcg4, Hgij, Hcij);

		Mat Hcg5(4, 4, CV_64FC1);
		Kron_HandEye(Hcg5, Hgij, Hcij);

		hand_eye_fout << "相机坐标系到手爪坐标系的转换关系" << endl;
		hand_eye_fout << "Tsai_HandEye;" << endl;
		hand_eye_fout << Hcg1 << endl << endl;
		hand_eye_fout << "DualQ_HandEye;" << endl;
		hand_eye_fout << Hcg2 << endl << endl;
		hand_eye_fout << "Inria_HandEye;" << endl;
		hand_eye_fout << Hcg3 << endl << endl;
		hand_eye_fout << "Navy_HandEye;" << endl;
		hand_eye_fout << Hcg4 << endl << endl;
		hand_eye_fout << "Kron_HandEye;" << endl;
		hand_eye_fout << Hcg5 << endl << endl << endl;

		Mat Hbe;         //机械臂基座标系到相机坐标系的关系
		hand_eye_fout << "机械臂基座标系到相机坐标系的关系" << endl;
        hand_eye_fout << "Tsai_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			cout<<"Hbt"<<i<<Hbt[i]<<endl;
			cout<<"Hcg1.inv()"<<Hcg1.inv()<<endl;
			cout<<"eye2hand[i+1].inv()"<<eye2hand[i+1].inv()<<endl;
			hand_eye_fout << Hbt[i]*Hcg1*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "DualQ_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg2*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "Inria_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg3*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "Navy_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg4*eye2hand[i+1].inv() << endl << endl;
		}
		hand_eye_fout << "Kron_HandEye;" << endl;
		for(int i=0;i<Hbt.size();i++)
		{
			hand_eye_fout << Hbt[i]*Hcg5*eye2hand[i+1].inv() << endl << endl;
		}
		
	}

	cout << "手眼标定完成" << endl;
	return 0;
}
