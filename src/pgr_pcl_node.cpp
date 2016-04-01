#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>

#include <stdio.h>
#include <stdlib.h>
#include "triclops.h"
#include "fc2triclops.h"

#include "stereoto3dpoints.h"
#include "image_publisher.h"

int
main( int argc, char** argv )
{
    TriclopsInput triclopsColorInput, triclopsMonoInput;
    TriclopsContext triclops;

    FC2::Camera camera;
    FC2::Image grabbedImage;
    FC2::Error fc2Error;
    FC2::CameraInfo camInfo;

    camera.Connect();

    // configure camera
    if ( configureCamera( camera ) )
    {
        return EXIT_FAILURE;
    }

    // generate the Triclops context
    if ( generateTriclopsContext( camera, triclops ) )
    {
        return EXIT_FAILURE;
    }

    // Get the camera info and print it out
    fc2Error = camera.GetCameraInfo( &camInfo );

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        ROS_INFO( "Failed to get camera info from camera\n" );
        exit( -1 );
    }
    else
    {
        ROS_INFO( ">>>>> CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d  Resolution: %s", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber, camInfo.sensorResolution );
    }

    // Part 1 of 2 for grabImage method
    fc2Error = camera.StartCapture();

    if ( fc2Error != FC2::PGRERROR_OK )
    {
        exit( FC2T::handleFc2Error( fc2Error ) );
    }

    // Start Ros Node
    ros::init( argc, argv, "camera_system" );
    ros::NodeHandle nh;
    // Container of Images used for processing
    image_transport::ImageTransport it( nh );
    //Publishers for the camera
    image_transport::Publisher image_pub_left = it.advertise( "/camera/left/rgb", 1 );
    image_transport::Publisher image_pub_right = it.advertise( "/camera/right/rgb", 1 );
    image_transport::Publisher image_pub_disparity = it.advertise( "/camera/disparity", 1 );
    ros::Publisher pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>( "/vision3D/points", 0 );

    //+=+=+=+=+=+=+=+=+=+=+=+=  WHILE LOOP +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=
    ros::Rate loop_rate( 30 );

    while ( ros::ok() )
    {
        // this image contains both right and left images
        fc2Error = camera.RetrieveBuffer( &grabbedImage );

        if ( fc2Error != FC2::PGRERROR_OK )
        {
            exit( FC2T::handleFc2Error( fc2Error ) );
        }

        // Container of Images used for processing
        ImageContainer imageContainer;

        // generate triclops inputs from grabbed image
        if ( generateTriclopsInput( grabbedImage,
                                    imageContainer,
                                    triclopsColorInput,
                                    triclopsMonoInput )
           )
        {
            return EXIT_FAILURE;
        }

        FC2T::ErrorType fc2TriclopsError;
        fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                               grabbedImage,
                               true /*assume little endian*/,
                               imageContainer.unprocessed[RIGHT],
                               imageContainer.unprocessed[LEFT] );

        if ( fc2TriclopsError != FC2T::ERRORTYPE_OK )
        {
            FC2T::handleFc2TriclopsError( fc2TriclopsError, "unpackUnprocessedRawOrMono16Image" );
        }

        // Convert FC2::Image to BGR format
        convertToBGR( imageContainer.unprocessed[1], imageContainer.bgr[1] );



        // output image disparity image with subpixel interpolation
        TriclopsImage16 disparityImage16;

        // carry out the stereo pipeline
        if ( doStereo( triclops, triclopsMonoInput, disparityImage16 ) )
        {
            return EXIT_FAILURE;
        }

        // Publish images
        ImagePublisher imagePublisher( grabbedImage, imageContainer, &( image_pub_left ), &( image_pub_right ) );

        PointCloud cloud3d;

        // save text file containing 3d points
        if ( do3dPoints( grabbedImage, triclops, disparityImage16, triclopsColorInput, cloud3d ) )

        {
            return EXIT_FAILURE;
        }

        cloud3d.header.frame_id = "bumblebee2";
        cloud3d.header.stamp = ros::Time::now().toNSec();
        pointCloudPublisher.publish( cloud3d );
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Close the camera and disconnect
    camera.StopCapture();
    camera.Disconnect();

    // Destroy the Triclops context
    TriclopsError te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );

    return 0;
}
