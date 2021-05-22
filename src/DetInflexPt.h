#pragma once
/*Detect inflexion points*/

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<algorithm>

using namespace std;
using namespace cv;
struct InflexionPt
{
	vector<vector<Point> > new_edgeList;
	vector<vector<Point> > new_segList;
};

InflexionPt* detectInflexPt(vector<vector<Point>> edgeList, vector<vector<Point> >segList)
{
	InflexionPt* result = new InflexionPt;
	int no_seg_grps = segList.size();

	vector<vector<Point>> tempSegList, tempEdgeList;
	for (int ii = 0; ii < no_seg_grps; ii++)
	{
		vector<Point> present_seg_grp;
		vector<int> break_at_seg, break_points_y_x_list;
		present_seg_grp = segList[ii];
		if (present_seg_grp.size() <= 4)
		{
			
			tempSegList.push_back(segList[ii]);
			tempEdgeList.push_back(edgeList[ii]);
			continue;
		}//endif

		// slope angle	
		vector<float> theta, theta_2pi;
		for (int jj = 0; jj < present_seg_grp.size() - 1; jj++)// adjoint angles
		{
			float tempAngle = atan2(present_seg_grp[jj + 1].x - present_seg_grp[jj].x, present_seg_grp[jj + 1].y - present_seg_grp[jj].y);
			theta.push_back(tempAngle);
			if (tempAngle < 0)
			{
				theta_2pi.push_back(tempAngle + 2 * CV_PI);
			}//endif
			else
			{
				theta_2pi.push_back(tempAngle);
			}
		}//endfor

		// angle differences
		vector<float> theta_diff, theta_diff_2pi;
		for (int kk = 0; kk < theta.size() - 1; kk++)
		{
			theta_diff.push_back(theta[kk] - theta[kk + 1]);
			theta_diff_2pi.push_back(theta_2pi[kk] - theta_2pi[kk + 1]);
		}//endfor
		// correct angles
		for (int i = 0; i < theta_diff.size(); i++)
		{
			if (theta_diff[i]<-1 * CV_PI || theta_diff[i]> CV_PI)
			{
				theta_diff[i] = theta_diff_2pi[i];
			}//endif
		}//endfor
		//polarity
		vector<int> polarity;
		for (int i = 0; i < theta_diff.size(); i++)
		{
			if (theta_diff[i] > 0)
			{
				polarity.push_back(1);
			}//endif
			else { polarity.push_back(0); }//endelse
		}//endfor


		int count = 0;// non zero polarity
		int polaritySize = polarity.size();
		for (int i = 0; i < polaritySize; i++)
		{
			if (polarity[i] == 1) count++;
		}//endfor
		
		if (count > (double)(polaritySize / 2))
		{
			
			for (int j = 0; j < polaritySize; j++)
			{
				if (polarity[j] == 0)
				{
					
					polarity[j] = 1;
				}
				else
				{
					polarity[j] = 0;
				}
			}//endfor
		}//endif


		//checking ...0 1 0...type inflexion
		vector<int> mask1;
		mask1.push_back(0);
		mask1.push_back(1);
		mask1.push_back(0);
		int location = 0;
		while (location < (polaritySize - 2))
		{
			
			vector<int> window;
			window.push_back(polarity[location]);
			window.push_back(polarity[location + 1]);
			window.push_back(polarity[location + 2]);
			vector<int> is_inflexion;
			// xor, result is stored in is_inflexion
			for (int i = 0; i < 3; i++)
			{
				if (window[i] - mask1[i] == 0) { is_inflexion.push_back(0); }
				else
				{
					is_inflexion.push_back(1);
				}
			}//endfor
			vector<int>::iterator pos;
			pos = find(is_inflexion.begin(), is_inflexion.end(), 1);
			if (pos != is_inflexion.end())
			{
				location += 1;
			}
			else
			{
				polarity[location] = 0;
				polarity[location + 1] = 0;
				polarity[location + 2] = 0;
				break_at_seg.push_back(location + 2);
				location += 2;
			}//endifelse
		}//endwhile


		//checking ... 0 0 1 1 ...type inflexion
		vector<int> mask2;
		mask2.push_back(0);
		mask2.push_back(1);
		mask2.push_back(1);
		mask2.push_back(0);
		int location2 = 0;
		while (location2 < polaritySize - 3)
		{
			vector<int> window;
			window.push_back(polarity[location2]);
			window.push_back(polarity[location2 + 1]);
			window.push_back(polarity[location2 + 2]);
			window.push_back(polarity[location2 + 3]);
			vector<int> is_inflexion;
			// xor, result is stored in is_inflexion
			for (int i = 0; i < 4; i++)
			{
				if (window[i] - mask2[i] == 0) { is_inflexion.push_back(0); }
				else {
					is_inflexion.push_back(1);
				}
			}//endfor
			vector<int>::iterator pos;
			pos = find(is_inflexion.begin(), is_inflexion.end(), 1);
			if (pos != is_inflexion.end())
			{
				location2 += 1;
			}
			else
			{
				
				polarity[location2] = 0;
				polarity[location2 + 1] = 0;
				polarity[location2 + 2] = 0;
				polarity[location2 + 3] = 0;
				break_at_seg.push_back(location2 + 2);
				break_at_seg.push_back(location2 + 3);
				location2 += 3;
			}//endifelse
		}//endwhile

		vector<int> transitions;
		vector<int> locations;
		// xor, result is stored in is_inflexion
		for (int i = 0; i < polaritySize - 1; i++)
		{
			if ((polarity[i] - polarity[i + 1]) == 0)
			{
				transitions.push_back(0);
			}
			else {
				transitions.push_back(1);
				locations.push_back(i); // record the index of nonzero elements
			}
		}//endfor

		// checking 1 0... type inflexion (the start segment is off)
		if ((!locations.empty()) && (locations.front() == 0))
		{
			
			break_at_seg.push_back(1);
			locations.erase(locations.begin());
			
		}//endif

		//checking ... 0 1 type inflexion(the end segment is off)
		if ((!locations.empty()) && (locations.back() == (polaritySize - 2)))//*(locations.end()-1
		{
			
			break_at_seg.push_back(present_seg_grp.size() - 2);
			locations.pop_back();
		}

		//checking ...0 1 1 1...type inflexion
		if (!locations.empty())
		{
			for (auto it = locations.begin(); it != locations.end(); it++)
				break_at_seg.push_back((*it) + 1);
		}//endif

		//now_breaking the edges and seglists
		if (break_at_seg.empty())
		{
			tempSegList.push_back(segList[ii]);
			tempEdgeList.push_back(edgeList[ii]);
		}//endif
		else
		{
			sort(break_at_seg.begin(), break_at_seg.end());// sort in the ascending order
			int breakAtSegSize = break_at_seg.size();
			for (int jj = 0; jj < breakAtSegSize; jj++)
			{
			
				int x1 = present_seg_grp[break_at_seg[jj]].x;
				int y1 = present_seg_grp[break_at_seg[jj]].y;
				
				vector<int> a;
				
				for (int kk = 0; kk < edgeList[ii].size(); kk++)
				{
					if ((edgeList[ii][kk].x == x1) && (edgeList[ii][kk].y == y1))
					{
						
						a.push_back(kk);
					}//endif
				}//endfor
				if (!a.empty())
				{
					
					break_points_y_x_list.push_back(a[0]);
				}//endif
			}//endfor
			


			// for segList
			vector<vector<Point> >seglist_temp;
			int k = 0;
			for (int jj = 0; jj < breakAtSegSize; jj++)
			{
				vector<Point> seg_temp;
				for (int p = k; p <= break_at_seg[jj]; p++)
				{
					seg_temp.push_back(segList[ii][p]);
				}
				seglist_temp.push_back(seg_temp);
				k = break_at_seg[jj];
			}
			vector<Point> seg_temp;
			for (int pp = k; pp < segList[ii].size(); pp++)// the final segment
			{
				seg_temp.push_back(segList[ii][pp]);
			}//endfor
			seglist_temp.push_back(seg_temp);
			for (int ppp = 0; ppp < seglist_temp.size(); ppp++)
			{
				tempSegList.push_back(seglist_temp[ppp]);
			}//endfor

			// for edgeList
			vector<vector<Point> >edgelist_temp;
			int k2 = 0;
			if (!break_points_y_x_list.empty())
			{
				for (int jj = 0; jj < break_points_y_x_list.size(); jj++)
				{
					vector<Point> edge_temp;
					for (int p = k2; p <= break_points_y_x_list[jj]; p++)
					{
						edge_temp.push_back(edgeList[ii][p]);
					}
					edgelist_temp.push_back(edge_temp);
					k2 = break_points_y_x_list[jj];
				}
				vector<Point> edge_temp2;
				for (int pp = k2; pp < edgeList[ii].size(); pp++)// the final segment
				{
					edge_temp2.push_back(edgeList[ii][pp]);
				}//endfor
				edgelist_temp.push_back(edge_temp2);
				for (int ppp = 0; ppp < edgelist_temp.size(); ppp++)
				{
					tempEdgeList.push_back(edgelist_temp[ppp]);
				}//endfor
			}//endif
		}//endelse
	}
	result->new_edgeList = tempEdgeList;
	result->new_segList = tempSegList;
	return result;

}//endfunction
