#include <frontier_exploration/frontier_search.h>

#include <costmap_2d/costmap_2d.h>
#include<costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>
#include <math.h>       //wlf add
#include <frontier_exploration/costmap_tools.h>
#include <frontier_exploration/Frontier.h>

namespace frontier_exploration{

using costmap_2d::LETHAL_OBSTACLE;//254
using costmap_2d::NO_INFORMATION;//255
using costmap_2d::FREE_SPACE;//0
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE; //jim253

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D &costmap) : costmap_(costmap) { }

std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position , unsigned int& index , visualization_msgs::MarkerArray& markerArray){
    ROS_INFO("searchFrom------------------------------------------");
    std::list<Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching检查机器人是否在costmap边界内
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
		ROS_INFO("position.x=%lf,position.y=%lf,originx=%lf,originy=%lf",position.x,position.y,costmap_.origin_x_,costmap_.origin_y_);
		ROS_INFO("end_x=%lf,end_y=%lf",costmap_.origin_x_+costmap_.size_x_*costmap_.resolution_,costmap_.origin_y_+costmap_.size_y_*costmap_.resolution_);
        return frontier_list;
    }
    ros::NodeHandle n;
    //marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1000);
    //visualization_msgs::MarkerArray markerArray;
    number = 0;         //wlf add
    //make sure map is consistent and locked for duration of search  
    //boost::unique_lock < boost::shared_mutex > lock(*(costmap_.getLock()));  // For old version of costmap api
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex())); // new version of costmap api
    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells初始化标记数组
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //initialize breadth first search初始化广度优先搜索
    std::queue<unsigned int> bfs;

    //find closest clear cell to start search
    unsigned int clear, pos = costmap_.getIndex(mx,my);
    if(nearestCell(clear, pos, FREE_SPACE, costmap_)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    //bfs.push(pos);        //wlf add
    visited_flag[bfs.front()] = true;
    double min_dis_total=0;
    double grid_num_total=0;
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();//弹出用过的栅格

        //iterate over 4-connected neighbourhood四邻域搜索
        BOOST_FOREACH(unsigned nbr, nhood4(idx, costmap_))
	{
            //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
            //if(map_[nbr] <= map_[idx] && !visited_flag[nbr])
	    //if(!visited_flag[nbr] && map_[nbr] >= FREE_SPACE && map_[nbr] <= INSCRIBED_INFLATED_OBSTACLE ) // jim  0<=nbr<253
	    if(!visited_flag[nbr] && map_[nbr] == FREE_SPACE && !isNewFrontierCell(nbr,frontier_flag)) // jim  0<=nbr<253
	    //if(!visited_flag[nbr] && map_[nbr] == FREE_SPACE) // jim  0<=nbr<253
	    {
                visited_flag[nbr] = true;
                bfs.push(nbr);    //压入新的栅格
                //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
            }else if(isNewFrontierCell(nbr, frontier_flag)){
                frontier_flag[nbr] = true;
                visualization_msgs::Marker Circle;
                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag,Circle);
                /*if(new_frontier.size > 1){
                    frontier_list.push_back(new_frontier);
                    markerArray.markers.push_back(Circle);
                    number++;
                }*/
                if(Circle.scale.x > 0.10 && new_frontier.grid_num > 2){    //如果圆的直径大于60cm，则保留对应的边界
				
                    frontier_list.push_back(new_frontier);
                    markerArray.markers.push_back(Circle);
                    number++;
                    min_dis_total += 1/new_frontier.min_distance;
                    grid_num_total += (double)new_frontier.grid_num;
                }
            }
        }
    }
//添加评判函数
        double cost=0,temp,jj=0;
        BOOST_FOREACH(Frontier frontier, frontier_list){
            temp = (1/frontier.min_distance)/min_dis_total + 6*frontier.grid_num/grid_num_total;
            //temp = frontier.grid_num/grid_num_total;
            //temp = (1/frontier.min_distance)/min_dis_total;
            frontier.cost = temp;
            //check if this frontier is the nearest to robot找到离机器人最近的边界
            //if (temp > cost){
            //    cost = temp;
            //    index=jj;
            //}
            //jj++;
        }
        //markerArray.markers[index].color.r = 100;

/////////////////
    //marker_pub.publish(markerArray);              //发布
    return frontier_list;

}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag,visualization_msgs::Marker& circle){
//initial_cell为边界初始链接点，reference为机器人位置
    //initialize frontier structure
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier   边界的初始链接点，由这个点扩散出去
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    costmap_.mapToWorld(ix,iy,output.initial.x,output.initial.y);
    //geometry_msgs::Point temp;
    //temp.x=output.initial.x;
    //temp.y=output.initial.y;
    output.points[0].x=output.initial.x;
    output.points[0].y=output.initial.y;
    //push initial gridcell onto queue初始化队列
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);
/*
    while(!bfs.empty()){   //沿着边界栅格，进行广域优先搜索
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);

                //update frontier size
                output.size++;

                //update centroid of frontier 
                output.centroid.x += wx;
                output.centroid.y += wy;

                //determine frontier's distance from robot, going by closest gridcell to robot边界到机器人的最短距离
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }
*/
    double cx,cy;
    double nx,ny;
    double rr,cenx,ceny,rd;
    while(!bfs.empty()){   //沿着边界栅格，进行广域优先搜索
        unsigned int idx = bfs.front();
        bfs.pop();
 
        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);
                if(output.size==1){
                    cx=wx;
                    cy=wy;
                }
                else if(output.size==2){
                    nx=wx;
                    ny=wy;
                    rr=sqrt((cx-nx)*(cx-nx) + (cy-ny)*(cy-ny))/2;
                    cenx = (cx+nx)/2;
                    ceny = (cy+ny)/2;
                }
                else if(output.size >2){
                    rd=sqrt((wx-cenx)*(wx-cenx) + (wy-ceny)*(wy-ceny));
                    if(rd>rr){
                        rr=(rr+rd)/2;
                        cenx=cenx+(rd-rr)/rd*(wx-cenx);
                        ceny=ceny+(rd-rr)/rd*(wy-ceny);
                    }
                }
            //ROS_INFO("~~~~~~~~~~~~~~");
                //temp.x=wx;
                //temp.y=wy;
                //output.temp.push_back(temp);
                
                //update frontier size
                output.size++;
                output.points[output.size-1].x=wx;
                output.points[output.size-1].y=wy;
                //update centroid of frontier 
                output.centroid.x += wx;
                output.centroid.y += wy;

                //determine frontier's distance from robot, going by closest gridcell to robot边界到机器人的最短距离
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance && distance > 0.3){
                    output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }

                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

//计算圆内的未知栅格个数
    std::vector<bool> visited_flag1(size_x_ * size_y_, false);
    std::queue<unsigned int> bfs1;
    unsigned int cx1,cy1,grid_number=0;
    costmap_.worldToMap(cenx,ceny,cx1,cy1);
    unsigned int cen_pos = costmap_.getIndex(cx1,cy1);
    bfs1.push(cen_pos);
    visited_flag1[cen_pos]=true;
    while(!bfs1.empty()){   //沿着边界栅格，进行广域优先搜索
        unsigned int idx = bfs1.front();
        bfs1.pop();
 
        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //check if neighbour is a potential frontier cell
                unsigned int mx1,my1;
                double wx1,wy1;
                costmap_.indexToCells(nbr,mx1,my1);
                costmap_.mapToWorld(mx1,my1,wx1,wy1);
            if(sqrt((wx1-cenx)*(wx1-cenx)+(wy1-ceny)*(wy1-ceny)) < rr && !visited_flag1[nbr]){
                grid_number++;
                visited_flag1[nbr]=true;
                bfs1.push(nbr);
            }
        }
    }
    output.grid_num = grid_number;
    ROS_INFO("number=%d",grid_number);






//发布圆
	 	circle.header.frame_id = "/map";
	 	circle.header.stamp = ros::Time();
	 	circle.type =visualization_msgs::Marker::SPHERE;
	 	circle.action = visualization_msgs::Marker::ADD;
	 	/*marker1.scale.x = 1;
	 	marker1.scale.y = 1;
	 	marker1.scale.z =1;
*/
		circle.scale.x = 2*rr;
	 	circle.scale.y = 2*rr;
	 	circle.scale.z = 0;

	 /*	marker1.scale.x = 2;
	 	marker1.scale.y = 2;
	 	marker1.scale.z = 2;*/

	 	circle.color.a = 1.0;
	 	circle.color.r=0.0;
	 	circle.color.g=0.0;
	 	circle.color.b=1.0;

	 	circle.ns = "circle";
	 	circle.id = number;
	 	//std::cout<<marker1.id<<std::endl;
	 	//circle.text = ss.str();
	 	//std::cout<<marker1.text<<std::endl;
	 	circle.pose.position.x = cenx;
	 	circle.pose.position.y = ceny;
	 	// marker1.pose.position.z = 4;

	 	circle.pose.position.z = 0;
	 	circle.pose.orientation.x = 0.0;
   		circle.pose.orientation.y = 0.0;
   		circle.pose.orientation.z = 0.0;
   		circle.pose.orientation.w = 1.0;
    //average out frontier centroid边界的平均点位置
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    return output;   //！！！！！！！！！！！！！！！！！！！！！改成存储边界中所有的点
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //check that cell is unknown and not already marked as frontier
    if(map_[idx] != FREE_SPACE || frontier_flag[idx]){
    //if(map_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free  idx为255，且其周围至少有一个自由空间的栅格
    BOOST_FOREACH(unsigned int nbr, nhood4(idx, costmap_)){
        if(map_[nbr] == NO_INFORMATION){
        //if(map_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;

}

}
