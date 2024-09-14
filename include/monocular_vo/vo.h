
#include <iostream>
#include <string>

class VisualOdometry
{
    private:
     std::string dataset_path;
    public :
      VisualOdometry(std::string dataset_path);
    
};

VisualOdometry::VisualOdometry(std::string dataset_path)
{
    this->dataset_path = dataset_path;
    std::cout << "Dataset path: " << dataset_path << std::endl;
}
