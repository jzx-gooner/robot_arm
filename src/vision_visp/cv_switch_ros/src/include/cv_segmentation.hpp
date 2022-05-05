#ifndef CV_SEGMENTATION_H
#define CV_SEGMENTATION_H

#include "infer/trt_infer.hpp"
#include "common/preprocess_kernel.cuh"
#include "common/ilogger.hpp"


using namespace TRT;
using namespace std;


class CvSegmentation {

public:

    cv::Mat image_to_tensor(const cv::Mat& image, std::shared_ptr<TRT::Tensor>& tensor, int ibatch);

    std::tuple<cv::Mat, cv::Mat> post_process(std::shared_ptr<TRT::Tensor>& tensor, int ibatch);

    void render(cv::Mat& image, const cv::Mat& prob, const cv::Mat& iclass);

    cv::Mat inference(cv::Mat& image);

    void getEngine( const string& model_file);



private:

    std::shared_ptr<Infer> engine_ = nullptr;


};
#endif