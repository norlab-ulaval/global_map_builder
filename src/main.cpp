#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <unordered_map>
#include <norlab_icp_mapper_ros/SaveMap.h>

const int CELL_SIZE = 20;

typedef PointMatcher<float> PM;
ros::Publisher publisher;
PM::DataPoints globalMap;
bool globalMapEmpty = true;

int toGridCoordinate(const float& worldCoordinate)
{
	return std::floor(worldCoordinate / CELL_SIZE);
}

std::unordered_map<std::string, PM::DataPoints> splitIntoCells(const PM::DataPoints& cloud)
{
	std::unordered_map<std::string, PM::DataPoints> cells;
	std::unordered_map<std::string, int> cellPointCounts;
	for(int i = 0; i < cloud.getNbPoints(); i++)
	{
		int row = toGridCoordinate(cloud.features(0, i));
		int column = toGridCoordinate(cloud.features(1, i));
		int aisle = toGridCoordinate(cloud.features(2, i));
		std::string cellId = std::to_string(row) + "_" + std::to_string(column) + "_" + std::to_string(aisle);

		if(cells[cellId].getNbPoints() == 0)
		{
			cells[cellId] = cloud.createSimilarEmpty();
		}

		cells[cellId].setColFrom(cellPointCounts[cellId], cloud, i);
		cellPointCounts[cellId]++;
	}
	for(auto& cell: cells)
	{
		cell.second.conservativeResize(cellPointCounts[cell.first]);
	}
	return cells;
}

float toInferiorWorldCoordinate(const int& gridCoordinate)
{
	return gridCoordinate * CELL_SIZE;
}

float toSuperiorWorldCoordinate(const int& gridCoordinate)
{
	return (gridCoordinate + 1) * CELL_SIZE;
}

void deleteCellFromGlobalMap(const int& row, const int& column, const int& aisle)
{
	float startX = toInferiorWorldCoordinate(row);
	float endX = toSuperiorWorldCoordinate(row);
	float startY = toInferiorWorldCoordinate(column);
	float endY = toSuperiorWorldCoordinate(column);
	float startZ = toInferiorWorldCoordinate(aisle);
	float endZ = toSuperiorWorldCoordinate(aisle);

	int globalMapNbPoints = 0;
	for(int i = 0; i < globalMap.features.cols(); i++)
	{
		if(globalMap.features(0, i) < startX || globalMap.features(0, i) >= endX || globalMap.features(1, i) < startY ||
		   globalMap.features(1, i) >= endY || globalMap.features(2, i) < startZ || globalMap.features(2, i) >= endZ)
		{
			globalMap.setColFrom(globalMapNbPoints, globalMap, i);
			globalMapNbPoints++;
		}
	}
	globalMap.conservativeResize(globalMapNbPoints);
	globalMapEmpty = globalMap.getNbPoints() == 0;
}

std::tuple<int, int, int> extractGridCoordinatesFromString(const std::string& stringCoordinates)
{
	size_t currentPos = stringCoordinates.find("_");
	int row = std::stoi(stringCoordinates.substr(0, currentPos));
	size_t lastPos = currentPos + 1;
	currentPos = stringCoordinates.find("_", lastPos);
	int column = std::stoi(stringCoordinates.substr(lastPos, currentPos - lastPos));
	lastPos = currentPos + 1;
	int aisle = std::stoi(stringCoordinates.substr(lastPos));
	return std::make_tuple(row, column, aisle);
}

void localMapCallback(const sensor_msgs::PointCloud2& msg)
{
	PM::DataPoints localMap = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msg);
	if(!globalMapEmpty)
	{
		std::unordered_map<std::string, PM::DataPoints> cells = splitIntoCells(localMap);
		for(auto& cell: cells)
		{
			std::tuple<int, int, int> gridCoordinates = extractGridCoordinatesFromString(cell.first);
			deleteCellFromGlobalMap(std::get<0>(gridCoordinates), std::get<1>(gridCoordinates), std::get<2>(gridCoordinates));
			globalMap.concatenate(cell.second);
		}
	}
	else
	{
		globalMap = localMap;
	}
	globalMapEmpty = globalMap.getNbPoints() == 0;
	publisher.publish(PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(globalMap, msg.header.frame_id, msg.header.stamp));
}

bool saveGlobalMapCallback(norlab_icp_mapper_ros::SaveMapRequest& request, norlab_icp_mapper_ros::SaveMapResponse& response)
{
	try
	{
		globalMap.save(request.map_file_name.data);
		return true;
	}
	catch(const std::runtime_error& e)
	{
		ROS_ERROR_STREAM("Unable to save: " << e.what());
		return false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_map_builder_node");
	ros::NodeHandle nodeHandle;

	publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
	ros::Subscriber subscriber = nodeHandle.subscribe("map", 1, localMapCallback);
	ros::ServiceServer serviceServer = nodeHandle.advertiseService("save_global_map", saveGlobalMapCallback);

	ros::spin();

	return 0;
}
