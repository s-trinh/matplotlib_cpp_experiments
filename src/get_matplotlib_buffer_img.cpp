#include <matplotlibcpp.h>
#include <opencv2/opencv.hpp>

using namespace std;
namespace plt = matplotlibcpp;

// https://github.com/lava/matplotlib-cpp/issues/195#issuecomment-2731542163
// Is it possible to draw plot into simple buffer? #195
int main() {
  try {
    // Prepare data
    int ncols = 500, nrows = 300;
    std::vector<float> z(ncols * nrows);
    for (int j=0; j<nrows; ++j) {
        for (int i=0; i<ncols; ++i) {
            z.at(ncols * j + i) = std::sin(std::hypot(i - ncols/2, j - nrows/2));
        }
    }

    const float* zptr = &(z[0]);
    const int colors = 1;

    plt::title("My matrix");
    plt::imshow(zptr, nrows, ncols, colors);

    // Call the save_to_buffer function
    std::vector<uint8_t> buffer = plt::save_to_buffer();

    // Decode the buffer into an OpenCV Mat object
    cv::Mat image = cv::imdecode(buffer, cv::IMREAD_COLOR);
    if (image.empty()) {
        throw std::runtime_error("Failed to decode image from buffer");
    }

    // Display the image using OpenCV
    cv::imshow("Plot", image);
    cv::waitKey(0);
  } catch (const std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl;
      return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}