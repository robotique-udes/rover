#include "bms_node.hpp"



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BMSDataNode>(argc, argv);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

BMSDataNode::BMSDataNode(int argc, char** argv):
    Node("bms_info")
{
    _publisher = this->create_publisher<rover_msgs::msg::BmsData>(TOPIC_BMS_DATA, QOS_DEFAULT);

    _timer_publisher = this->create_wall_timer(std::chrono::milliseconds(DELAY_PUBLISHER_MS),
                                                [this](void)
                                                {
                                                    this->callbackBMSData();
                                                });
}

int BMSDataNode::getData()
{

    std::ifstream file(DATA_FILE_PATH);
    std::string line;

    if (!file.is_open())
    {
        std::cout << "Error" << std::endl;
        return 1;
    }

    for(uint8_t i=0;i<4;i++)
    {
        uint8_t counter = 0;

        getline(file, line);
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

        if(i==1 || i==3)
        {
            while(line[counter] != '=')
            {
                counter++;
            }
            
            if(i==1)
            {
                std::string stringNumber;

                while(counter + 1 < line.size())
                {
                    stringNumber += line[counter+1];
                    counter++;
                }
                _batteryAmps = std::stoi(stringNumber);
                std::cout << _batteryAmps << std::endl;
            }
            else if(i==3)
            {
                for(uint8_t cellIndex = 0;cellIndex<MAX_CELL;cellIndex++)
                {
                    std::string stringNumber;

                    while(line[counter+1] != ':' && line[counter+1] != ' ')
                    {
                        stringNumber+= line[counter+1];
                        counter++;
                    }
                    counter++;
                    std::cout << stringNumber << std::endl;
                    _cellVolt[cellIndex] = std::stoi(stringNumber);
                    std::cout << _cellVolt[cellIndex] << std::endl;
                }
                

            }
        }

    }

    file.close();

    return 0;
}

void BMSDataNode::callbackBMSData()
{
    rover_msgs::msg::BmsData msg;

    getData();
}