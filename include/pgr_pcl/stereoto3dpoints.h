#ifndef STEREOTO3DPOINTS_H
#define STEREOTO3DPOINTS_H

#include "triclops.h"
#include "fc2triclops.h"
#include <stdio.h>
#include <stdlib.h>
#include <pcl_ros/point_cloud.h>
#include "common.h"
#include "typedefs.h"

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// configue camera to capture image
int configureCamera( FC2::Camera &camera );

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera     & camera,
                             TriclopsContext & triclops );

// capture image from connected camera
int grabImage( FC2::Camera & camera, FC2::Image & grabbedImage );

// convert image to BRGU
int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage,
                           ImageContainer   & imageContainer,
                           TriclopsInput    & colorData,
                           TriclopsInput    & stereoData );

// carry out stereo processing pipeline
int doStereo( TriclopsContext const & triclops,
              TriclopsInput  const & stereoData,
              TriclopsImage16      & depthImage );

// save 3d points generated from stereo processing
int do3dPoints( FC2::Image      const & grabbedImage,
                TriclopsContext const & triclops,
                TriclopsImage16 const & disparityImage16,
                TriclopsInput   const & colorData ,
                PointCloud &returnedPoints );

// save 3d points generated from stereo processing
int save3dPoints( FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData );



#endif // STEREOTO3DPOINTS_H
