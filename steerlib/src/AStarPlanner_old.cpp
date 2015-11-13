//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define INT_MAX       2147483647  
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		int X = gSpatialDatabase->getNumCellsX();
		int Z = gSpatialDatabase->getNumCellsZ();
		float cellsizeX = gSpatialDatabase->getCellSizeX();
		float cellsizeZ = gSpatialDatabase->getCellSizeZ();

		std::vector<std::vector<AStarPlannerNode>> gridcell;
		gridcell.resize(X);
		for (size_t i = 0; i < X; i++ )
			gridcell[i].resize(Z);

		unsigned int index_s = gSpatialDatabase->getCellIndexFromLocation(start);
		unsigned int x_s, z_s;
		gSpatialDatabase->getGridCoordinatesFromIndex(index_s, x_s, z_s);
		unsigned int index_g = gSpatialDatabase->getCellIndexFromLocation(goal);
		unsigned int x_g, z_g;
		gSpatialDatabase->getGridCoordinatesFromIndex(index_g, x_g, z_g);

		std::vector<AStarPlannerNode> open;
		std::vector<AStarPlannerNode> closed;
		
		for (int i = 0; i < X; i++)
			for (int j = 0; j < Z; j++)
			{
				gridcell[i][j].g = INT_MAX;
			}

		gridcell[x_s][z_s].g = 0.0f;
		gridcell[x_s][z_s].h = calcH(start, goal);
		gridcell[x_s][z_s].f = gridcell[x_s][z_s].g + gridcell[x_s][z_s].h;
		gridcell[x_s][z_s].point = start;

		open.push_back(gridcell[x_s][z_s]);

		double smallest_F = gridcell[x_s][z_s].f;

		std::cout << " X and Z " << " " << X << " " << Z << std::endl;
		std::cout << " start " << " " << start.x << " " << start.z << std::endl;
		std::cout << " start " << " " << x_s << " " << z_s << std::endl;
		std::cout << " goal " << " " << goal.x << " " << goal.z << std::endl;
		std::cout << " goal " << " " << x_g << " " << z_g << std::endl;
		std::cout << " gridcell[x_g][z_g].g " << " " << gridcell[x_g][z_g].g << std::endl;
		std::cout << " smallest_F " << " " << smallest_F << std::endl;
		std::cout << " cellsizeX " << " " << cellsizeX << std::endl;
		std::cout << " cellsizeZ " << " " << cellsizeZ << std::endl;

		int iit = 0;
		int loop = 0;
		while (gridcell[x_g][z_g].g > smallest_F)
		{	
			closed.push_back(open[iit]);
			open.erase(open.begin() + iit);
			std::cout << " loop" << loop << std::endl;
			std::cout << " open_size" << open.size() << std::endl;

			//update surrounding cells cost
			unsigned int index_n = gSpatialDatabase->getCellIndexFromLocation(open[iit].point);
			unsigned int x_n, z_n;
			gSpatialDatabase->getGridCoordinatesFromIndex(index_n, x_n, z_n);
			int x_range_min, x_range_max, z_range_min, z_range_max;
			x_range_min = MAX(x_n - OBSTACLE_CLEARANCE, 0);
			x_range_max = MIN(x_n + OBSTACLE_CLEARANCE, X);
			z_range_min = MAX(z_n - OBSTACLE_CLEARANCE, 0);
			z_range_max = MIN(z_n + OBSTACLE_CLEARANCE, Z);

			//down
			if(x_range_max == x_n + 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n + 1, z_n);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n + 1][z_n].g > gridcell[x_n][z_n].g + cellsizeX)
					{
						gridcell[x_n + 1][z_n].g = gridcell[x_n][z_n].g + cellsizeX;
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n + 1][z_n].point = current_position;
						gridcell[x_n + 1][z_n].h = calcH(current_position, goal);
						gridcell[x_n + 1][z_n].parent = &gridcell[x_n][z_n];
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n + 1][z_n]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n + 1][z_n]), open.end());
						}
						open.push_back(gridcell[x_n + 1][z_n]);
					}
				}
			}

			//up
			if (x_range_min == x_n - 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n - 1, z_n);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n - 1][z_n].g > gridcell[x_n][z_n].g + cellsizeX)
					{
						gridcell[x_n - 1][z_n].g = gridcell[x_n][z_n].g + cellsizeX;
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n - 1][z_n].point = current_position;
						gridcell[x_n - 1][z_n].h = calcH(current_position, goal);
						gridcell[x_n - 1][z_n].parent = &gridcell[x_n][z_n];
						gridcell[x_n - 1][z_n].f = gridcell[x_n - 1][z_n].g + gridcell[x_n - 1][z_n].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n - 1][z_n]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n - 1][z_n]), open.end());
						}
						open.push_back(gridcell[x_n - 1][z_n]);
					}
				}
			}

			//right
			if (z_range_max == z_n + 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n, z_n + 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n][z_n + 1].g > gridcell[x_n][z_n].g + cellsizeZ)
					{
						gridcell[x_n][z_n + 1].g = gridcell[x_n][z_n].g + cellsizeZ;
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n][z_n + 1].point = current_position;
						gridcell[x_n][z_n + 1].h = calcH(current_position, goal);
						gridcell[x_n][z_n + 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n][z_n + 1].f = gridcell[x_n][z_n + 1].g + gridcell[x_n][z_n + 1].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n][z_n + 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n][z_n + 1]), open.end());
						}
						open.push_back(gridcell[x_n][z_n + 1]);
					}
				}
			}

			//left
			if (z_range_min == z_n - 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n, z_n - 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n][z_n - 1].g > gridcell[x_n][z_n].g + cellsizeZ)
					{
						gridcell[x_n][z_n - 1].g = gridcell[x_n][z_n].g + cellsizeZ;
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n][z_n - 1].point = current_position;
						gridcell[x_n][z_n - 1].h = calcH(current_position, goal);
						gridcell[x_n][z_n - 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n][z_n - 1].f = gridcell[x_n][z_n - 1].g + gridcell[x_n][z_n - 1].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n][z_n - 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n][z_n - 1]), open.end());
						}
						open.push_back(gridcell[x_n][z_n - 1]);
					}
				}
			}

			//down-left
			if (x_range_max == x_n + 1 && z_range_min == z_n - 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n + 1, z_n - 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n + 1][z_n - 1].g > gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ))
					{
						gridcell[x_n + 1][z_n - 1].g = gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n + 1][z_n - 1].point = current_position;
						gridcell[x_n + 1][z_n - 1].h = calcH(current_position, goal);
						gridcell[x_n + 1][z_n - 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n + 1][z_n - 1].f = gridcell[x_n + 1][z_n - 1].g + gridcell[x_n + 1][z_n - 1].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n + 1][z_n - 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n + 1][z_n - 1]), open.end());
						}
						open.push_back(gridcell[x_n + 1][z_n - 1]);
					}
				}
			}

			//up-left
			if (x_range_min == x_n - 1 && z_range_min == z_n - 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n - 1, z_n - 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n - 1][z_n - 1].g > gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ))
					{
						gridcell[x_n - 1][z_n - 1].g = gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n - 1][z_n - 1].point = current_position;
						gridcell[x_n - 1][z_n - 1].h = calcH(current_position, goal);
						gridcell[x_n - 1][z_n - 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n - 1][z_n - 1].f = gridcell[x_n - 1][z_n - 1].g + gridcell[x_n - 1][z_n - 1].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n - 1][z_n - 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n - 1][z_n - 1]), open.end());
						}
						open.push_back(gridcell[x_n - 1][z_n - 1]);
					}
				}
			}

			//up-right
			if (x_range_min == x_n - 1 && z_range_max == z_n + 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n - 1, z_n + 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n - 1][z_n + 1].g > gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ))
					{
						gridcell[x_n - 1][z_n + 1].g = gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n - 1][z_n + 1].point = current_position;
						gridcell[x_n - 1][z_n + 1].h = calcH(current_position, goal);
						gridcell[x_n - 1][z_n + 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n - 1][z_n + 1].f = gridcell[x_n - 1][z_n + 1].g + gridcell[x_n - 1][z_n + 1].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n - 1][z_n + 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n - 1][z_n + 1]), open.end());
						}
						open.push_back(gridcell[x_n - 1][z_n + 1]);
					}
				}
			}

			//down-right
			if (x_range_max == x_n + 1 && z_range_max == z_n + 1)
			{
				unsigned int index_neighbour = gSpatialDatabase->getCellIndexFromGridCoords(x_n + 1, z_n + 1);
				if (canBeTraversed(index_neighbour))
				{
					if (gridcell[x_n + 1][z_n + 1].g > gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ))
					{
						gridcell[x_n + 1][z_n + 1].g = gridcell[x_n][z_n].g + std::sqrt(cellsizeX*cellsizeX + cellsizeZ*cellsizeZ);
						Util::Point current_position;
						gSpatialDatabase->getLocationFromIndex(index_neighbour, current_position);
						gridcell[x_n + 1][z_n + 1].point = current_position;
						gridcell[x_n + 1][z_n + 1].h = calcH(current_position, goal);
						gridcell[x_n + 1][z_n + 1].parent = &gridcell[x_n][z_n];
						gridcell[x_n + 1][z_n + 1].f = gridcell[x_n + 1][z_n + 1].g + gridcell[x_n + 1][z_n + 1].h;
						//delete from the list if already presented
						if (std::find(open.begin(), open.end(), gridcell[x_n + 1][z_n + 1]) != open.end())
						{
							open.erase(std::remove(open.begin(), open.end(), gridcell[x_n + 1][z_n + 1]), open.end());
						}
						open.push_back(gridcell[x_n + 1][z_n + 1]);
					}
				}
			}

			std::vector<AStarPlannerNode>::iterator min;
			min = std::min_element(open.begin(), open.end(), [](auto &a, auto &b) {return a.f < b.f; });
			iit = min - open.begin();
			smallest_F = open[iit].f;
			loop++;
		}

		if (open.empty() == true)
		{
			std::cout << " cant reach the target" << std::endl;
			return false;
		}

		AStarPlannerNode node = gridcell[x_g][z_g];
		std::vector<AStarPlannerNode> temp;
		temp.push_back(node);
		while (node.point != gridcell[x_s][z_s].point)
		{
			node = (*gridcell[x_g][z_g].parent);
			temp.push_back((node));
		}

		agent_path.push_back(start);
		for (auto i = temp.end() - 1; i > temp.begin() - 1; i--)
		{
			agent_path.push_back((*i).point);
		}


		//std::cout<<"\nIn A*";
		return true;
	}

	double AStarPlanner::calcH(Util::Point p, Util::Point goal)
	{
		return (std::abs(p.x - goal.x) + std::abs(p.z - goal.z));
	}

}