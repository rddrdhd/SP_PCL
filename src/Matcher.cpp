//
// Created by rddrdhd on 21.5.22.
//

#include "Matcher.h"

Matcher::Matcher(pcl::PointCloud<PointType>::Ptr s,
                 pcl::PointCloud<PointType>::Ptr m,
                 pcl::PointCloud<PointType>::Ptr mk,
                 pcl::PointCloud<PointType>::Ptr sk,
                 pcl::CorrespondencesPtr c) {
    this->scene_ = s;
    this->model_ = m;
    this->model_keypoints_ = mk;
    this->scene_keypoints_ = sk;
    this->correspondences_ = c;


}

void Matcher::GCClustering(float cg_size , float cg_thresh){
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size);
    gc_clusterer.setGCThreshold (cg_thresh);

    gc_clusterer.setInputCloud (this->model_keypoints_);
    gc_clusterer.setSceneCloud (this->scene_keypoints_);
    gc_clusterer.setModelSceneCorrespondences (this->correspondences_);
    gc_clusterer.cluster (clustered_corrs);
    //gc_clusterer.recognize (rototranslations, clustered_corrs);
    this->rototranslations_ = rototranslations;
    this->clustered_corrs_ = clustered_corrs;
}
void Matcher::output() {
    std::cout << "Model instances found: " << this->rototranslations_.size () << std::endl;
    for (std::size_t i = 0; i < this->rototranslations_.size(); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs_[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = this->rototranslations_[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = this->rototranslations_[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }
}
