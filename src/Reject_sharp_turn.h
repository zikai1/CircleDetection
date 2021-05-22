#pragma once
//Reject sharp turn angles

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<math.h>

using namespace std;
using namespace cv;


struct sharpTurn
{
	vector<vector<Point>> new_edgeList;
	vector<vector<Point>> new_segList;
};


sharpTurn* rejectSharpTurn(vector<vector<Point>> edgeList, vector<vector<Point> >segList, float angle)
{
	sharpTurn* result = new sharpTurn;
	int no_seg_grps = segList.size();
	vector<Point>break_points;
	double Threshold_theta = CV_PI / 2;
	vector<vector<Point> > new_segList1, new_edgeList1;
	

	for (int ii = 0; ii < no_seg_grps; ii++)//no_seg_grps
	{
		
		vector<Point> present_seg_grp = segList[ii];
		int no_of_seg = present_seg_grp.size() - 1;//seg_point -1=no_of_seg;
		// use cos(angle) calculation to determine sharp turn angles
		Point present_vector = Point(present_seg_grp[1].x - present_seg_grp[0].x, present_seg_grp[1].y - present_seg_grp[0].y);
		

		for (int seg_no = 0; seg_no < no_of_seg - 1; seg_no++)
		{
			double length_present_vector = pow(pow(present_vector.x, 2) + pow(present_vector.y, 2), 0.5);
			Point next_vector = Point(present_seg_grp[seg_no + 2].x - present_seg_grp[seg_no + 1].x, present_seg_grp[seg_no + 2].y - present_seg_grp[seg_no + 1].y);
			double length_next_vector = pow(pow(next_vector.x, 2) + pow(next_vector.y, 2), 0.5);
			double cos_pre_next = (present_vector.x * next_vector.x + present_vector.y * next_vector.y) / (length_present_vector * length_next_vector);
			
			if (cos_pre_next <= cos(angle / 180.0 * CV_PI))
			{
				// check again if it is true sharp turns
				Point temp = Point(ii, seg_no + 1);// record the seg_no+1 node of ii-th segList
				break_points.push_back(temp);

			}
			present_vector = next_vector;
		}//endfor
	}

	if (break_points.empty())//no break points
	{
		result->new_edgeList = edgeList;
		result->new_segList = segList;
		return result;
	}
	int index = 0;
	int current_break = break_points[index].x;
	for (int ii = 0; ii < no_seg_grps; ii++)
	{
		vector<Point> current_seg = segList[ii];
		vector<Point> current_edge = edgeList[ii];
		if (current_seg.size() > 2)
		{
			if (ii == current_break)
			{
				int count = 1;
				int first_edge_index, last_edge_index, first_seg_index, last_seg_index;
				while (ii == current_break)
				{
					if (count == 1)
					{
						first_seg_index = 0;
						first_edge_index = 0;
					}
					else
					{
						first_seg_index = last_seg_index;
						first_edge_index = last_edge_index;
					}
					last_seg_index = break_points[index].y;

					for (int jj = first_edge_index; jj < current_edge.size(); jj++)
					{
						if ((current_seg[last_seg_index].x == current_edge[jj].x) && (current_seg[last_seg_index].y == current_edge[jj].y))
						{

							last_edge_index = jj;
							break;
						}//endif
					}//endfor
					//block before break
					//cut block




					if ((last_seg_index - first_seg_index) >= 1)
					{
						vector<Point> block_seg, block_edge;
						for (int kk = first_seg_index; kk <= last_seg_index; kk++)
						{
							block_seg.push_back(current_seg[kk]);
						}
						for (int kk = first_edge_index; kk <= last_edge_index; kk++)
						{
							block_edge.push_back(current_edge[kk]);
						}
						new_edgeList1.push_back(block_edge);
						new_segList1.push_back(block_seg);
					}

					//check for break
					index += 1;
					if (index > (break_points.size() - 1))
					{
						break;
					}
					current_break = break_points[index].x;
					count = count + 1;

				}//endwhile
				//block after break
				if ((current_seg.size() - last_seg_index) >= 2)
				{
					vector<Point> block1_seg, block1_edge;
					for (int ll = last_seg_index; ll < current_seg.size(); ll++)
					{
						
						block1_seg.push_back(current_seg[ll]);
						
					}
					for (int ll = last_edge_index; ll < current_edge.size(); ll++)
					{
						
						block1_edge.push_back(current_edge[ll]);
					}
					new_edgeList1.push_back(block1_edge);
					new_segList1.push_back(block1_seg);
					
				}
			}//endif
			else
			{
				new_edgeList1.push_back(edgeList[ii]);
				new_segList1.push_back(segList[ii]);
				
			}//endelse
		}//endif
	}//endfor
	result->new_edgeList = new_edgeList1;
	result->new_segList = new_segList1;
	return result;
}







