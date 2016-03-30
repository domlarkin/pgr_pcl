#ifndef IMAGE_PUBLISHER_H
#define IMAGE_PUBLISHER_H
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "common.h"

/**
 * @brief The ImagePublisher class. More details coming soon!
 *
 */
int convertToBGR( FC2::Image & image, FC2::Image & convertedImage );


class ImagePublisher
    //Document this code with UML diagram generator online
    //Have diagram of desired outcome.
{
    public:

        ImagePublisher( FC2::Image grabbedImage, ImageContainer imageContainer, image_transport::Publisher *image_pub_left, image_transport::Publisher *image_pub_right );

    private:
        cv::Mat      leftImage;
        cv::Mat      rightImage;
};

#endif // IMAGE_PUBLISHER_H


