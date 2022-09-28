#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <unordered_map>
#include <norlab_icp_mapper_ros/srv/save_map.hpp>

class GlobalMapBuilderNode : public rclcpp::Node
{
public:
    GlobalMapBuilderNode():
            Node("global_map_builder_node")
    {
        publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 1);
        mapSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("map", 1, std::bind(&GlobalMapBuilderNode::localMapCallback, this,
                                                                                                       std::placeholders::_1));
        saveGlobalMapService = this->create_service<norlab_icp_mapper_ros::srv::SaveMap>("save_global_map",
                                                                                         std::bind(&GlobalMapBuilderNode::saveGlobalMapCallback, this,
                                                                                                   std::placeholders::_1, std::placeholders::_2));
    }

private:
    const int CELL_SIZE = 20;

    typedef PointMatcher<float> PM;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSubscription;
    rclcpp::Service<norlab_icp_mapper_ros::srv::SaveMap>::SharedPtr saveGlobalMapService;
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

    void replaceCellInGlobalMap(const int& row, const int& column, const int& aisle, const PM::DataPoints& newCell)
    {
        float startX = toInferiorWorldCoordinate(row);
        float endX = toSuperiorWorldCoordinate(row);
        float startY = toInferiorWorldCoordinate(column);
        float endY = toSuperiorWorldCoordinate(column);
        float startZ = toInferiorWorldCoordinate(aisle);
        float endZ = toSuperiorWorldCoordinate(aisle);

        int globalMapNbPoints = 0;
        int oldChunkNbPoints = 0;
        PM::DataPoints oldChunk = globalMap.createSimilarEmpty();
        for(int i = 0; i < globalMap.features.cols(); i++)
        {
            if(globalMap.features(0, i) >= startX && globalMap.features(0, i) < endX && globalMap.features(1, i) >= startY &&
               globalMap.features(1, i) < endY && globalMap.features(2, i) >= startZ && globalMap.features(2, i) < endZ)
            {
                oldChunk.setColFrom(oldChunkNbPoints, globalMap, i);
                oldChunkNbPoints++;
            }
            else
            {
                globalMap.setColFrom(globalMapNbPoints, globalMap, i);
                globalMapNbPoints++;
            }
        }
        globalMap.conservativeResize(globalMapNbPoints);
        oldChunk.conservativeResize(oldChunkNbPoints);

        if((float)oldChunkNbPoints / (float)newCell.getNbPoints() >= 100.0f)
        {
            // probably an error, do not remove old chunk
            globalMap.concatenate(oldChunk);
        }

        globalMap.concatenate(newCell);
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

    void localMapCallback(const sensor_msgs::msg::PointCloud2& msg)
    {
        PM::DataPoints localMap = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(msg);
        if(!globalMapEmpty)
        {
            std::unordered_map<std::string, PM::DataPoints> cells = splitIntoCells(localMap);
            for(auto& cell: cells)
            {
                std::tuple<int, int, int> gridCoordinates = extractGridCoordinatesFromString(cell.first);
                replaceCellInGlobalMap(std::get<0>(gridCoordinates), std::get<1>(gridCoordinates), std::get<2>(gridCoordinates), cell.second);
            }
        }
        else
        {
            globalMap = localMap;
        }
        globalMapEmpty = globalMap.getNbPoints() == 0;
        publisher->publish(PointMatcher_ROS::pointMatcherCloudToRosMsg<float>(globalMap, msg.header.frame_id, msg.header.stamp));
    }

    void saveGlobalMapCallback(const std::shared_ptr<norlab_icp_mapper_ros::srv::SaveMap::Request> request,
                               std::shared_ptr<norlab_icp_mapper_ros::srv::SaveMap::Response> response)
    {
        try
        {
            globalMap.save(request->map_file_name.data);
        }
        catch(const std::runtime_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to save: %s", e.what());
        }
    }
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMapBuilderNode>());
    rclcpp::shutdown();
    return 0;
}
