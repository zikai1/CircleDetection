/**************************************************************************************************************
* An occlusion-resistant circle detector using inscribed triangles source code.
* Copyright (c) 2021, Mingyang Zhao
* E-mails of the authors: zhaomingyang16@mails.ucas.ac.cn
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

* By using this implementation, please cite the following paper:
*
* M. Zhao, X. Jia, D.Y "An occlusion-resistant circle detector using inscribed triangles,"
*     Pattern Recognition (2021).
**************************************************************************************************************/

#include "EDLib.h"
#include <iostream>
#include<opencv2/imgproc.hpp>

using namespace cv;
using namespace std;


/*---set thresholds---*/
// For better performance, you can mainly tune the parameters:  'T_inlier' and 'sharp_angle' 
typedef struct threshold {
	int T_l = 20;
	float T_ratio = 0.001;
	int T_o = 5;// 5 10 15 20 25
	int T_r = 5;// 5 10 15 20 25
	float T_inlier = 0.35;//0.3 0.35 0.4 0.45 0.5 (the larger the more strict)
	float T_angle = 2.0;// 
	float T_inlier_closed = 0.5;//0.5,0.6 0.7 0.8,0.9
	float sharp_angle = 60;//35 40 45 50 55 60 

}T;

int main()
{
	
	// You should create at least two directories: 'Images1' & 'result'
	// If you have the ground truth, you can create the directory  'GT'
	cv::String path = "E:/Code/patterns/Images1/";
	cv::String dst = "E:/Code/patterns/result/";
	//cv::String GT = "D:/astudy/dataset/circle/temp/GT/";


	T test_threshold;

	vector<cv::String> Filenames;
	cv::glob(path, Filenames);
	float fmeasureSum = 0.0;
	float precisionSum = 0.0;
	float recallSum = 0.0;
	float timeSum = 0.0;

	// Detect each image in the directory 'Images1'
	for (int i = 0; i < Filenames.size(); i++)
	{
		//read images
		cv::String file = Filenames[i];
		cv::String::size_type pos1, pos2;
		pos1 = file.find("1");
		pos2 = file.find(".");
		cv::String prefix = file.substr(pos1 + 2, pos2 - pos1 - 2);
		cv::String suffix = file.substr(pos2 + 1, pos2 + 3);

		//the name of saved detected images
		cv::String saveName = dst + prefix + "_det." + suffix;

	   // Gaussian denoise (optional), we do not use in paper
		Mat testImgOrigin = imread(file, 1);//0:gray 1:color	
		Mat testImg = testImgOrigin.clone();
		cvtColor(testImg, testImg, COLOR_BGR2GRAY);
		GaussianBlur(testImg, testImg, Size(9, 9), 2, 2);
		cv::imshow("Clone Image", testImg);
		waitKey();
		int height = testImg.rows;
		int width = testImg.cols;

		

		
		/*---------Illustration for each step---------*/
		Mat test1 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test2 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test3 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test4 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test5 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test6 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test7 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test8 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test9 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test10 = Mat(height, width, CV_8UC1, Scalar(255));
		Mat test11 = Mat(height, width, CV_8UC1, Scalar(255));


		// EDPF Parameter-free Edge Segment Detection 
		clock_t start, finish;
		start = clock();
		
		EDPF testEDPF = EDPF(testImg);
		Mat edgePFImage = testEDPF.getEdgeImage();
		Mat edge = edgePFImage.clone();
		edge = edge * -1 + 255;
		cv::imshow("Edge Image Parameter Free", edge);
		//imwrite("D:/astudy/dataset/circle/temp/result/edge.jpg", edge);
		waitKey();
		vector<vector<Point> >EDPFsegments = testEDPF.getSegments();// get edge segments

		//plot edge images
		cvtColor(test10, test10, COLOR_GRAY2BGR);
		for (int es1 = 0; es1 < EDPFsegments.size(); es1++)
		{
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			Scalar SegEdgesColor = Scalar(b, g, r);
			for (int es2 = 0; es2 < EDPFsegments[es1].size() - 1; es2++)
			{
				cv::line(test10, EDPFsegments[es1][es2], EDPFsegments[es1][es2 + 1], SegEdgesColor, 2);//Scalar(0, 0, 0)
			}
		}
		imshow("Edge Segments image", test10);
		waitKey();
		//imwrite("D:/Test/temp/result/edge_segment.jpg", test10);*/


		/*--------delete edge segments whose pixel number is less than 16-------------*/
		vector<vector<Point>> edgeList;
		for (int i = 0; i < EDPFsegments.size(); i++)
		{
			if (EDPFsegments[i].size() >= 16)// segments should have at least 16 pixels
			{
				edgeList.push_back(EDPFsegments[i]);
			}//endif
		}//endfor


		/*----------extract closed edges-------------------*/
		closedEdgesExtract* closedAndNotClosedEdges;
		closedAndNotClosedEdges = extractClosedEdges(edgeList);
		vector<vector<Point> > closedEdgeList;
		closedEdgeList = closedAndNotClosedEdges->closedEdges;

		/*--------approximate edge segments using line segments by method RDP-------*/
		vector<vector<Point> > segList;
		for (int s0 = 0; s0 < edgeList.size(); s0++)
		{
			vector<Point> segTemp;
			RamerDouglasPeucker(edgeList[s0], 2.5, segTemp);//3.0
			segList.push_back(segTemp);
		}

		/*-------------reject sharp turn angles---------------*/
		sharpTurn* newSegEdgeList;
		newSegEdgeList = rejectSharpTurn(edgeList, segList, test_threshold.sharp_angle);
		
		//new seglist and edgelist
		vector<vector<Point>> newSegList = newSegEdgeList->new_segList;
		vector<vector<Point>> newEdgeList = newSegEdgeList->new_edgeList;

		//plot segLists after sharp turn splitting
		cvtColor(test2, test2, COLOR_GRAY2BGR);
		for (int j = 0; j < newSegList.size(); j++)
		{
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			Scalar colorSharpTurn = Scalar(b, g, r);

			for (int jj2 = 0; jj2 < newEdgeList[j].size() - 1; jj2++)
			{
				//circle(test2, newSegList[j][jj], 1, Scalar(0, 0, 0), 3);
				line(test2, newEdgeList[j][jj2], newEdgeList[j][jj2 + 1], colorSharpTurn, 2);
			}

		}
		imshow("After sharp turn", test2);
		waitKey();
		//imwrite("sharpTurn.jpg", test2);



		/*-----------------Detect inflexion points--------------*/

		InflexionPt* newSegEdgeListAfterInflexion;
		newSegEdgeListAfterInflexion = detectInflexPt(newEdgeList, newSegList);
		
		// new seglist and edgelist
		vector<vector<Point>> newSegListAfterInflexion = newSegEdgeListAfterInflexion->new_segList;
		vector<vector<Point>> newEdgeListAfterInfexion = newSegEdgeListAfterInflexion->new_edgeList;


		/*--------delete short edgeLists or near line segments----------*/
		vector<vector<Point>>::iterator it = newEdgeListAfterInfexion.begin();
		while (it != newEdgeListAfterInfexion.end())
		{
			/*compute the line segment generated by the two endpoints of the arc,
			and then judge the midpoint of the arc if lying on or near the line
			*/
			Point edgeSt = Point((*it).front().x, (*it).front().y);
			Point edgeEd = Point((*it).back().x, (*it).back().y);
			int midIndex = (*it).size() / 2;

			Point edgeMid = Point((*it)[midIndex].x, (*it)[midIndex].y);

			double distStEd = sqrt(pow(edgeSt.x - edgeEd.x, 2) + pow(edgeSt.y - edgeEd.y, 2));
			double distStMid = sqrt(pow(edgeSt.x - edgeMid.x, 2) + pow(edgeSt.y - edgeMid.y, 2));
			double distMidEd = sqrt(pow(edgeEd.x - edgeMid.x, 2) + pow(edgeEd.y - edgeMid.y, 2));
			double distDifference = abs((distStMid + distMidEd) - distStEd);


			if ((*it).size() <= test_threshold.T_l || distDifference <= test_threshold.T_ratio * (distStMid + distMidEd))// 2 3 fixed number; (*it).size() <=20
			{
				it = newEdgeListAfterInfexion.erase(it);
			}
			else { it++; }
		}//endwhile

		cvtColor(test11, test11, COLOR_GRAY2BGR);
		for (int j = 0; j < newEdgeListAfterInfexion.size(); j++)
		{
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			Scalar colorAfterDeleteLinePt = Scalar(b, g, r);
			for (int jj2 = 0; jj2 < newEdgeListAfterInfexion[j].size() - 1; jj2++)
			{
				//circle(test2, newSegList[j][jj], 1, Scalar(0, 0, 0), 3);
				line(test11, newEdgeListAfterInfexion[j][jj2], newEdgeListAfterInfexion[j][jj2 + 1], colorAfterDeleteLinePt, 2);
			}
		}
		imshow("After short and line segments remove", test11);
		waitKey();
		//imwrite("D:/Test/temp/result/remove_short_line.jpg", test3);


		/*-----extract closed edgeLists and not closed edgeLists after inflexion point operation------*/
		closedEdgesExtract* closedAndNotClosedEdges1;
		closedAndNotClosedEdges1 = extractClosedEdges(newEdgeListAfterInfexion);
		vector<vector<Point> > closedEdgeList1;
		vector<vector<Point> > notclosedEdgeList1;
		closedEdgeList1 = closedAndNotClosedEdges1->closedEdges;
		notclosedEdgeList1 = closedAndNotClosedEdges1->notClosedEdges;

		//plot closed edgeLists
		cvtColor(test4, test4, COLOR_GRAY2BGR);
		for (int j = 0; j < closedEdgeList1.size(); j++)
		{
			int r = rand() % 256;
			int g = rand() % 256;
			int b = rand() % 256;
			Scalar colorClosedEdges = Scalar(b, g, r);
			for (int jj = 0; jj < closedEdgeList1[j].size() - 1; jj++)
			{
				//circle(test4, newSegListAfterInflexion[j][jj], 1, Scalar(0, 0, 0), 3);
				cv::line(test4, closedEdgeList1[j][jj], closedEdgeList1[j][jj + 1], colorClosedEdges, 2);
			}
			//imshow("After infexion point remove", test2);
			//waitKey();
		}
		imshow("closedEdges2", test4);
		waitKey();
		//imwrite("closedEdges2.jpg", test4);

		/*----------sort notclosedEdgeList for grouping-------------*/
		std::vector<std::vector<Point>> sortedEdgeList = sortEdgeList(notclosedEdgeList1);

		/*--------------group sortededgeList---------------*/
		groupArcs* arcs = coCircleGroupArcs(sortedEdgeList, test_threshold.T_o, test_threshold.T_r);
		vector<vector<Point> > groupedArcs = arcs->arcsFromSameCircles;
		vector<vector<Point> > groupedArcsThreePt = arcs->arcsStartMidEnd;
		vector<Vec3f>  groupedOR = arcs->recordOR;


		/*--------circle verification using estimated center and radius parameters*/
		vector<Circles> groupedCircles;// grouped arcs
		groupedCircles = circleEstimateGroupedArcs(groupedArcs, groupedOR, groupedArcsThreePt, test_threshold.T_inlier, test_threshold.T_angle);//fit grouped arcs

		// closed arcs
		for (auto ite = closedEdgeList1.begin(); ite != closedEdgeList1.end(); ite++)
		{
			closedEdgeList.push_back(*ite);
		}//endfor


		vector<Circles> closedCircles;// closedCircles
		closedCircles = circleEstimateClosedArcs(closedEdgeList, test_threshold.T_inlier_closed);// fit closed edges


		//put grouped and closed circles together
		vector<Circles> totalCircles;
		if (!groupedCircles.empty())
		{
			totalCircles = groupedCircles;
		}
		if (!closedCircles.empty())
		{
			for (auto it = closedCircles.begin(); it != closedCircles.end(); it++)
			{
				totalCircles.push_back(*it);
			}
		}
		//cluster circles----------------->no clustering 
		finish = clock();
		vector<Circles> preCircles;
		preCircles = clusterCircles(totalCircles);
		//finish = clock();
		timeSum += ((float)(finish - start) / CLOCKS_PER_SEC);
		//draw fit circles after clustering
		Mat detectCircles = drawResult(true, testImgOrigin, saveName, preCircles);//totalCircles preCircles
	//}//endfor    run 100 times and then calculate the average


	
		/*-----compute precision, recall and fmeasure-------*/
		//pre_rec_fmeasure totalResult = Evaluate(gt, preCircles, 0.8f, testImg);
		//waitKey();
		////fmeasureSum += totalResult.fmeasure;
		//precisionSum += totalResult.precision;
		//recallSum += totalResult.recall;

	}

	float avePre = precisionSum / Filenames.size();//Filenames.size()
	float aveRec = recallSum / Filenames.size();//Filenames.size()
	float aveTime = timeSum / Filenames.size();
	float aveFmea = 2 * avePre * aveRec / (avePre + aveRec);
	cout << "Pre Rec Fmea Time: " << avePre << " " << aveRec << " " << aveFmea << " " << aveTime << endl;
	waitKey(0);

	return 0;
}



