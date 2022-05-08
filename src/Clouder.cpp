//
// Created by rddrdhd on 8.5.22.
//
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

    // WIP: fix the redundancy
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

void Clouder::showSHOTKeypoints(){
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

void Clouder::computeFPFHDescriptors() {
    pcl::console::TicToc tt; tt.tic();

    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh;

    //fpfh.setInputCloud (this->keypointsXYZ_); //?
    fpfh.setInputCloud (this->cloud_);
    fpfh.setInputNormals (this->normals_);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

    fpfh.setSearchMethod (tree);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setKSearch(descriptor_k_neighbours_);

    fpfh.compute (*features);
    this->FPFH_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Fast Persistent Feature Histogram takes %.3f\n[0]:\t", t );
    cout<<features->points[0]<<endl;
}

void Clouder::computePFHDescriptors(){ // TODO generating all-zero desriptors
    pcl::console::TicToc tt;tt.tic();
    pcl::PFHEstimation<PointType, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (this->cloud_);
    pfh.setInputNormals (this->normals_);
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType> ());
    pfh.setSearchMethod (tree);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr features (new pcl::PointCloud<pcl::PFHSignature125> ());

    pfh.setRadiusSearch (descriptor_radius_);
    //pfh.setKSearch(descriptor_k_neighbours_);

    // WIP: returning nans :(
    pfh.compute(*features);
    this->PFH_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Persistent Feature Histogram takes %.3f\n[0]:\t", t );
    cout << features->points[0] << endl;
}

void Clouder::computeSHOTDescriptors(){
    pcl::console::TicToc tt;tt.tic();

    // WIP: Local reference frames error
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr frames (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::SHOTLocalReferenceFrameEstimation<PointType, pcl::ReferenceFrame> lrf_estimator;
    lrf_estimator.setRadiusSearch (descriptor_radius_);
    lrf_estimator.setInputCloud (getKeypointsXYZ());
    lrf_estimator.setSearchSurface(cloud_);
    lrf_estimator.compute (*frames);


    pcl::SHOTEstimationOMP<PointType, pcl::Normal, pcl::SHOT352> shot;
    shot.setInputCloud (getKeypointsXYZ());
    shot.setInputNormals (normals_);
    shot.setSearchSurface(cloud_);
    shot.setRadiusSearch (descriptor_radius_);
    shot.setLRFRadius(rf_rad_);
    shot.setInputReferenceFrames(frames);
    // Output datasets
    pcl::PointCloud<pcl::SHOT352>::Ptr features (new pcl::PointCloud<pcl::SHOT352> ());
    // Compute the features
    shot.compute(*features);

    this->SHOT_descriptors_ = features;

    double t = tt.toc();
    pcl::console::print_value( "Signature of Histograms of Orientation takes %.3f\n", t );
    cout << features->points[0] << endl;
}
