// webots
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Lidar.hpp>
#include <webots/PositionSensor.hpp>
using namespace webots;

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <memory>

#define SAMPLE_PERIOD 50

int main( int argc, char* argv[] ) 
{   
    // connect to simulation
    Robot robot;

    Motor* yawMotor = robot.getMotor( "yaw motor" );
    Motor* pitchMotor = robot.getMotor( "pitch motor" );
    Motor* rollMotor = robot.getMotor( "roll motor" );
    
    const std::array<Motor*,3> motors { yawMotor, pitchMotor, rollMotor };
    for( auto m : motors )
    {
        if( !m ) return 1;

        auto p = m->getPositionSensor();
        if( !p ) return 2;
        p->enable( SAMPLE_PERIOD );   
    }

    Lidar* riLidar = robot.getLidar( "rangeimage lidar" );
    if( !riLidar ) return 3;
    riLidar->enable( SAMPLE_PERIOD );

    Lidar* pcLidar = robot.getLidar( "pointcloud lidar" );
    if( !pcLidar ) return 4;
    pcLidar->enable( SAMPLE_PERIOD );
    pcLidar->enablePointCloud();

    // setup for visualisation
    using point_t = pcl::PointXYZ;
    using cloud_t = pcl::PointCloud<point_t>;
    cloud_t::Ptr rangeImageCloud( new cloud_t );
    cloud_t::Ptr pointCloudCloud( new cloud_t );

    pcl::visualization::CloudViewer viewer( "Cloud Viewer" );

    // point at the pallets
    yawMotor->setPosition( M_PI/4 );
    pitchMotor->setPosition( 0 );
    rollMotor->setPosition( -0.57 );

    while( robot.step(SAMPLE_PERIOD) != -1 && !viewer.wasStopped() )
    {
        // compensate for roll
        const double th = rollMotor->getPositionSensor()->getValue();
        const double c = cos(th);
        const double s = sin(th);
        const std::array<double,9> rotationMatrix { c,-s, 0, 
                                                    s, c, 0, 
                                                    0, 0, 1 };

        std::cout << th << std::endl;

        /* Render the point cloud off the range image */
        struct IntrinsicMatrix
        {
            double x, y, fx, fy, s;
        } intrinsic;
        intrinsic.x = riLidar->getHorizontalResolution()*0.5;
        intrinsic.y = riLidar->getNumberOfLayers()*0.5;
        intrinsic.fx = intrinsic.x / tan( riLidar->getFov() *0.5 );
        intrinsic.fy = intrinsic.y / tan( riLidar->getVerticalFov() *0.5 );
        intrinsic.s = 0.0;

        rangeImageCloud->points.resize( riLidar->getHorizontalResolution() * riLidar->getNumberOfLayers() );

        const float* begin = riLidar->getRangeImage();
        const float* end = std::next( begin, riLidar->getHorizontalResolution() * riLidar->getNumberOfLayers() );

        std::transform( begin, end, 
                        rangeImageCloud->points.begin(), 
            [&]( const float& depth ) -> point_t
            {
                const int i = std::distance( begin, &depth );
                const int x = i % riLidar->getHorizontalResolution();
                const int y = i / riLidar->getHorizontalResolution();

                point_t point;
                point.x = (x - intrinsic.x) / intrinsic.fx * depth, 
                point.y = (intrinsic.y - y) / intrinsic.fy * depth, 
                point.z = -depth;

                point_t rotatedPoint;
                rotatedPoint.x = rotationMatrix[0]*point.x + rotationMatrix[1]*point.y + rotationMatrix[2]*point.z; 
                rotatedPoint.y = rotationMatrix[3]*point.x + rotationMatrix[4]*point.y + rotationMatrix[5]*point.z;
                rotatedPoint.z = rotationMatrix[6]*point.x + rotationMatrix[7]*point.y + rotationMatrix[8]*point.z;;

                return rotatedPoint;
            } );

        /* render the point cloud of the webots point cloud data */
        pointCloudCloud->points.resize( pcLidar->getNumberOfPoints() );

        const LidarPoint* pcBegin = pcLidar->getPointCloud();
        std::transform( pcBegin, std::next(pcBegin, pcLidar->getNumberOfPoints()),
                        pointCloudCloud->points.begin(), 
            [&]( const LidarPoint& p ) -> point_t
            {
                point_t point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;

                point_t rotatedPoint;
                rotatedPoint.x = rotationMatrix[0]*point.x + rotationMatrix[1]*point.y + rotationMatrix[2]*point.z; 
                rotatedPoint.y = rotationMatrix[3]*point.x + rotationMatrix[4]*point.y + rotationMatrix[5]*point.z;
                rotatedPoint.z = rotationMatrix[6]*point.x + rotationMatrix[7]*point.y + rotationMatrix[8]*point.z;;

                return rotatedPoint;
            } );

        //viewer.showCloud( rangeImageCloud );
        viewer.showCloud( pointCloudCloud );
    }

    return 0;
}
