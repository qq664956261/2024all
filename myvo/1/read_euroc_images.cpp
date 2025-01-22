#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

// Function to read the CSV file and extract timestamps and filenames
std::vector<std::pair<std::string, std::string>> readCSV(const std::string& csv_file) {
    std::vector<std::pair<std::string, std::string>> data;
    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "Error opening CSV file: " << csv_file << std::endl;
        return data;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Ignore comment lines
        if (line[0] == '#') continue;
        
        std::istringstream ss(line);
        std::string timestamp, filename;
        if (std::getline(ss, timestamp, ',') && std::getline(ss, filename)) {
            data.emplace_back(timestamp, filename);
        }
    }
    file.close();
    return data;
}

int main(int argc, char** argv) {
    // Check for correct usage
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_data_csv> <image_directory>" << std::endl;
        return -1;
    }

    // Input arguments
    std::string csv_file = argv[1];
    std::string image_dir = argv[2];

    // Read CSV file
    std::vector<std::pair<std::string, std::string>> data = readCSV(csv_file);
    if (data.empty()) {
        std::cerr << "No data read from CSV file." << std::endl;
        return -1;
    }

    // Loop through the data and load images
    for (const auto& entry : data) {
        std::string timestamp = entry.first;
        std::string filename = entry.second;
        std::string image_path = image_dir + "/" + timestamp + ".png";

        // Load the image using OpenCV
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Error loading image: " << image_path << std::endl;
            continue;
        }

        // Display image
        std::cout << "Displaying image at timestamp: " << timestamp << std::endl;
        cv::imshow("Image", image);
        cv::waitKey(10); // Wait for 10 ms per image
    }

    std::cout << "Finished processing images." << std::endl;
    return 0;
}
