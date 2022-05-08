//
// Created by rddrdhd on 8.5.22.
//
#include <pcl/features/fpfh.h>
#include "Clouder.h"

Clouder::Clouder() {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    this->cloud_ = cloud;
}
Clouder::Clouder(const char* filepath) {
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile (filepath, *cloud);
    this->cloud_ = cloud;
}

void Clouder::computeNormals() {
    pcl::console::TicToc tt; tt.tic();
    // Estimate the normals of the cloud_xyz
    pcl::NormalEstimation<PointType, pcl::PointNormal> ne;
    pcl::NormalEstimation<PointType, pcl::Normal> nee;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_point_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointType>::Ptr tree_n(new pcl::search::KdTree<PointType>());

    ne.setInputCloud(this->cloud_);
    ne.setSearchMethod(tree_n);
    ne.setKSearch(this->k_neighbours_);
    ne.compute(*cloud_point_normals);

    nee.setInputCloud(this->cloud_);
    nee.setSearchMethod(tree_n);
    nee.setKSearch(this->k_neighbours_);
    nee.compute(*cloud_normals);


    // Copy the xyz info from cloud_xyz and add it to cloud_point_normals as the xyz field in PointNormals estimation is zero
    for(size_t i = 0; i < cloud_point_normals->points.size(); ++i)
    {
        cloud_point_normals->points[i].x = this->cloud_->points[i].x;
        cloud_point_normals->points[i].y = this->cloud_->points[i].y;
        cloud_point_normals->points[i].z = this->cloud_->points[i].z;
    }
    this->point_normals_ = cloud_point_normals;
    this->normals_ = cloud_normals;

    double t = tt.toc();
    pcl::console::print_value( "Computing normals takes %.3f\n[0]:\t", t );
    cout<<cloud_normals->points[0]<<endl;
}

void Clouder::showNormals() {
    // Visualization of normals along with the original cloud
    pcl::visualization::PCLVisualizer viewer("Normals");
    viewer.setBackgroundColor( 0.0, 0.0, 0.5 );
    viewer.addPointCloudNormals<PointType, pcl::PointNormal> (cloud_, point_normals_, 1, 5, "normals");

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void Clouder::generateSIFTKeypoints() {

    pcl::console::TicToc tt;tt.tic();
    // Estimate the sift interest points using normals values from xyz as the Intensity variants
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree);
    sift.setMinimumContrast(this->min_contrast_);
    sift.setScales(this->min_scale_, this->n_octaves_, this->n_scales_per_octave_);

    sift.setInputCloud(this->point_normals_);
    sift.compute(this->keypoints_);

    double t = tt.toc();
    pcl::console::print_value( "Scale-invariant feature transform takes %.3f\n\tFrom %d to %d points\n", t, this->size(), this->getKeypointsXYZ()->size());
}

void Clouder::showKeypoints(){
    // Visualization of keypoints along with the original cloud
    auto keypointsXYZ = getKeypointsXYZ();

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> keypoints_color_handler (keypointsXYZ, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_color_handler (cloud_, 255, 0, 0);
    viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
    viewer.addPointCloud(cloud_, cloud_color_handler, "cloud");
    viewer.addPointCloud(keypointsXYZ, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

    while(!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void Clouder::generateDownsampledCloud(){
    pcl::console::TicToc tt;tt.tic();
    pcl::PointCloud<PointType>::Ptr result (new pcl::PointCloud<PointType> ());

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    uniform_sampling.setRadiusSearch (downsampling_radius);
    uniform_sampling.filter (*result);
    this->downsampled_cloud_ = result;
    double t = tt.toc();
    pcl::console::print_value( "Downsampling the cloud takes %.3f\n\tFrom %d to %d points\n", t, this->size(), this->downsampled_cloud_->size() );


}

void Clouder::generateFPFHDescriptors() {
    pcl::console::TicToc tt; tt.tic();

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;
    //fpfh.setInputCloud (this->keypointsXYZ_);
    fpfh.setInputCloud (this->cloud_);
    fpfh.setInputNormals (this->normals_);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setKSearch(descriptor_k_neighbours_);

    // Compute the features
    fpfh.compute (*features);
    this->FPFH_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Fast Persistent Feature Histogram takes %.3f\n[0]:\t", t );
    cout<<features->points[0]<<endl;
}


