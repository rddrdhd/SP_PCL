//
// Created by rddrdhd on 8.5.22.
//
#include "Clouder.h"

void Clouder::showKeypoints(){
    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints_color_handler (keypointsXYZ_, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color_handler (cloud_, 255, 0, 0);
    viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
    viewer.addPointCloud(cloud_, cloud_color_handler, "cloud");
    viewer.addPointCloud(keypointsXYZ_, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}
pcl::PointCloud<PointType>::Ptr Clouder::downsampleCloud(pcl::PointCloud<PointType>::Ptr cloud, float radius_search ){

    pcl::PointCloud<PointType>::Ptr result (new pcl::PointCloud<PointType> ());

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (cloud);
    uniform_sampling.setRadiusSearch (radius_search);
    uniform_sampling.filter (*result);

    return result;
}

void Clouder::generateNormals() {
    // Estimate the normals of the cloud_xyz
    pcl::NormalEstimation<PointType, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<PointType>::Ptr tree_n(new pcl::search::KdTree<PointType>());

    ne.setInputCloud(this->cloud_);
    ne.setSearchMethod(tree_n);
    //ne.setRadiusSearch(0.2);
    ne.setKSearch(this->k_neighbours_);
    ne.compute(*cloud_normals);

    // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i<cloud_normals->points.size(); ++i)
    {
        cloud_normals->points[i].x = this->cloud_->points[i].x;
        cloud_normals->points[i].y = this->cloud_->points[i].y;
        cloud_normals->points[i].z = this->cloud_->points[i].z;
    }
    this->normals_ = cloud_normals;
}
void Clouder::generateSIFTKeypoints() {

    pcl::console::TicToc tt;
    tt.tic();
    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setMinimumContrast(this->min_contrast_);
    sift.setScales(this->min_scale_, this->n_octaves_, this->n_scales_per_octave_);

    sift.setInputCloud(this->normals_);
    sift.compute(this->keypoints_);



    double t = tt.toc();
    pcl::console::print_value( "Scale-invariant feature transform takes %.3f\n[0]:\t", t );


    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
    copyPointCloud(this->keypoints_, *cloud_temp);
    this->keypointsXYZ_ = cloud_temp;
}

Clouder::Clouder(const char* filepath) {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile (filepath, *cloud);
    this->cloud_ = cloud;

}



void Clouder::showNormals() {
    // Visualization of keypoints along with the original cloud
    pcl::visualization::PCLVisualizer viewer("Normals");
    viewer.setBackgroundColor( 0.0, 0.0, 0.5 );
    viewer.addPointCloudNormals<PointType, pcl::PointNormal> (cloud_, normals_, 1, 5, "normals");

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}


