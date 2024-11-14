#include "p2pl_icp.h"
/*=============================================================================================================================

icp 2 plane: 3 dimension

=============================================================================================================================*/
scan2map3d::scan2map3d(pcl::PointCloud<PointType>::Ptr map_cloud, std::string yaml_path){
  map_kdtree_.reset(new nanoflann::KdTreeFLANN<PointType>());
  map_kdtree_->setInputCloud(map_cloud);

  map_cloud_.reset(new pcl::PointCloud<PointType>());
  pcl::copyPointCloud(*map_cloud, *map_cloud_);

  YAML::Node config;
  //读取yaml数据
  try{
    config = YAML::LoadFile(yaml_path);
  } catch(YAML::BadFile &e) {
    std::cout<<" Loc :  in process scan2map2d -- read config file error!"<<std::endl;
    return;
  }

  plane_threshold_ = config["plane_threshold"].as<float>();
  correspondences_threshold_normal_ = config["opt_correspondences_threshold_normal"].as<float>();
  correspondences_threshold_reloc_ = config["opt_correspondences_threshold_relocation"].as<float>();
  use_correspondences_threshold_ = correspondences_threshold_reloc_;
  
  max_iteration_ = config["opt_max_iteration"].as<int>();
  thread_num_ = config["thread_num"].as<int>();


  cloud_world.reset(new pcl::PointCloud<PointType>());
}

void scan2map3d::setRelocMode(void){
  use_correspondences_threshold_ = correspondences_threshold_reloc_;
}

void scan2map3d::setNormalMode(void){
  use_correspondences_threshold_ = correspondences_threshold_normal_;
}

void scan2map3d::setMaxIteration(int iter){
  max_iteration_ = iter;
}

void scan2map3d::setCorrespondencesThreshold(float value){
  use_correspondences_threshold_ = value;
}


void scan2map3d::surfOptimization(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud){
  Eigen::Matrix4f affine_mat = Pose2Matrix(init_pose);

  uint32_t pl_size = lidar_cloud->points.size();

  cloud_world->clear();
  PointType pt_world;
  for(uint32_t i=0; i<pl_size; i++){
    PointType pt = lidar_cloud->points[i];
    pt_world.x = affine_mat(0,0) * pt.x + affine_mat(0,1) * pt.y + affine_mat(0,2) * pt.z + affine_mat(0,3);
    pt_world.y = affine_mat(1,0) * pt.x + affine_mat(1,1) * pt.y + affine_mat(1,2) * pt.z + affine_mat(1,3);
    pt_world.z = affine_mat(2,0) * pt.x + affine_mat(2,1) * pt.y + affine_mat(2,2) * pt.z + affine_mat(2,3);
    pt_world.intensity = pt.intensity;

    cloud_world->points.push_back(pt_world);
  }

  std::vector<PointType> features(pl_size); //特征点
  std::vector<PointType> toplane_coffi(pl_size);
  std::vector<bool> picked(pl_size);



#pragma omp parallel for num_threads(thread_num_)
  //遍历每个激光点
  for(uint32_t i=0; i<pl_size; i++){
    PointType pointOri = lidar_cloud->points[i];
    PointType pointSel = cloud_world->points[i];

    PointType coeff;

    //kd树搜索结果
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    map_kdtree_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);


    Eigen::Matrix<float, 5, 3> matA0; //判断是否是平面的A矩阵
    Eigen::Matrix<float, 5, 1> matB0; //判断是否是平面的B矩阵
    Eigen::Vector3f matX0; //平面法向量

    matA0.setZero();
    matB0.fill(-1);
    matX0.setZero();

    //如果附近匹配点超过匹配范围，则跳过该点
    if(pointSearchInd.size() < 5){
      picked[i] = false;
      continue;
    } 
    if(pointSearchSqDis[4] > use_correspondences_threshold_){
      picked[i] = false;
      continue;
    }

    for (int j = 0; j < 5; j++) {
      matA0(j, 0) = map_cloud_->points[pointSearchInd[j]].x;
      matA0(j, 1) = map_cloud_->points[pointSearchInd[j]].y;
      matA0(j, 2) = map_cloud_->points[pointSearchInd[j]].z;
    }

    //matX0 = matA0.colPivHouseholderQr().solve(matB0);
    Eigen::Matrix<float, 3, 3> M = Eigen::Matrix<float, 3, 3>::Zero();
    Eigen::Matrix<float, 3, 1> N = Eigen::Matrix<float, 3, 1>::Zero();

    for(int i = 0; i < 5; i++){
      M(0,0) += matA0(i,0) * matA0(i,0);
      M(0,1) += matA0(i,0) * matA0(i,1);
      M(0,2) += matA0(i,0) * matA0(i,2);
      M(1,1) += matA0(i,1) * matA0(i,1);
      M(1,2) += matA0(i,1) * matA0(i,2);
      M(2,2) += matA0(i,2) * matA0(i,2);

      N(0) -= matA0(i,0);
      N(1) -= matA0(i,1);
      N(2) -= matA0(i,2);
    }

    M(1,0) = M(0,1);
    M(2,0) = M(0,2);
    M(2,1) = M(1,2);

    //Eigen::LLT<Eigen::Matrix<float, 3, 3>> lltOfA(M); // 计算A的LLT分解
    //matX0 = lltOfA.solve(N); // 求解线性方程组

    matX0 = M.llt().solve(N);

    //求出平面系数
    float pa = matX0(0, 0);
    float pb = matX0(1, 0);
    float pc = matX0(2, 0);
    float pd = 1.0;

    if(std::isnan(pa) || std::isnan(pb) || std::isnan(pc)){
      picked[i] = false;
      continue;
    } 


    float ps = sqrt(pa * pa + pb * pb + pc * pc);
    pa /= ps; pb /= ps; pc /= ps; pd /= ps;


    bool is_plane = true;
    for (int j = 0; j < 5; j++) {
      if (fabs(pa * map_cloud_->points[pointSearchInd[j]].x +
                pb * map_cloud_->points[pointSearchInd[j]].y +
                pc * map_cloud_->points[pointSearchInd[j]].z + pd) > plane_threshold_) {
        is_plane = false;
        break;
      }
    }

    if(is_plane==false){
      picked[i] = false;
      continue;
    } 
    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z));
    if(s < 0.1){
      picked[i] = false;
      continue;
    } 

    
    coeff.x = s * pa;
    coeff.y = s * pb;
    coeff.z = s * pc;
    coeff.intensity = s * pd2;

    features[i] = pointOri;
    toplane_coffi[i] = coeff;
    picked[i] = true;
  }


  features_.features.clear();
  features_.toplane_coffi.clear();
  features_.pickup_num = 0;

  for(uint32_t i=0; i<pl_size; i++){
    if(picked[i] == true){
      features_.features.push_back(features[i]);
      features_.toplane_coffi.push_back(toplane_coffi[i]);

      features_.pickup_num++;
    }
  }
}






bool scan2map3d::LM_optimization(int iter_count, pose_type &act_pose){

  uint32_t pl_size = features_.pickup_num;

  float srx = sin(act_pose.orient[0]);
  float crx = cos(act_pose.orient[0]);
  float sry = sin(act_pose.orient[1]);
  float cry = cos(act_pose.orient[1]);
  float srz = sin(act_pose.orient[2]);
  float crz = cos(act_pose.orient[2]);

  std::vector<Eigen::Matrix<double, 1, 6>> jacobians(pl_size);
  std::vector<double> errors(pl_size);


#pragma omp parallel for num_threads(thread_num_)
  for(uint32_t i=0; i<pl_size; i++){
    PointType pointOri = features_.features[i];
    PointType coeff = features_.toplane_coffi[i];
    //Jr计算
    float arx = ((crz * sry * crx + srz * srx)* pointOri.y - (crz * sry * srx - srz * crx) * pointOri.z) * coeff.x
                + ((srz * sry * crx - crz * srx) * pointOri.y - (srz * sry *srx + crz * crx) * pointOri.z) * coeff.y
                + (cry * crx * pointOri.y - cry * srx * pointOri.z) * coeff.z;


    float ary = (- crz * sry * pointOri.x + crz * cry * srx * pointOri.y  + crz * cry *crx * pointOri.z) * coeff.x
                + (-srz * sry * pointOri.x + srz * cry * srx * pointOri.y + srz * cry * crx * pointOri.z) * coeff.y 
                + (-cry * pointOri.x - srx * sry * pointOri.y - crx * sry * pointOri.z) * coeff.z;


    float arz = (-cry * srz * pointOri.x - (srx * sry * srz + crz * crx) * pointOri.y - (crx * sry * srz - crz * srx) * pointOri.z) * coeff.x
                + (cry * crz * pointOri.x + (srx * sry * crz - srz * crx) * pointOri.y + (crx * sry * crz + srz * srx) * pointOri.z) * coeff.y;


    Eigen::Matrix<double, 1, 6> J;
    J(0,0) = static_cast<double>(coeff.x);
    J(0,1) = static_cast<double>(coeff.y);
    J(0,2) = static_cast<double>(coeff.z);

    J(0,3) = static_cast<double>(arx);
    J(0,4) = static_cast<double>(ary);
    J(0,5) = static_cast<double>(arz);
    jacobians[i] = J;

    errors[i] = -coeff.intensity;
  }


  Eigen::Matrix<double, 6, 6> matAtA = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> matAtB = Eigen::Matrix<double, 6, 1>::Zero();

#pragma omp parallel for num_threads(thread_num_)
  for(uint32_t i=0; i<pl_size; i++){
      matAtA += jacobians[i].transpose() * jacobians[i];
      matAtB += jacobians[i].transpose() * errors[i];
  }



  Eigen::Matrix<double, 6,1> matX;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen_solver(matAtA);

  Eigen::Matrix<double, 6, 6> matP;
  Eigen::Matrix<double, 6, 1> matE; 
  Eigen::Matrix<double, 6, 6> matV; 
  Eigen::Matrix<double, 6, 6> matV2;
  Eigen::Matrix<double, 6, 1> matX2;  

  bool isDegenerate = false; 
  if (iter_count == 0){
    matE = eigen_solver.eigenvalues();
    matV = eigen_solver.eigenvectors();

    matV2 = matV;

    isDegenerate = false;
    double eignThre[6] = {100, 100, 100, 100, 100, 100};
    for (int i = 5; i >= 0; i--) {
      if (matE(i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate){ 
    matX2 = matX;
    matX = matP * matX2;
  }

  act_pose.pos(0)+=matX(0);
  act_pose.pos(1)+=matX(1);
  act_pose.pos(2)+=matX(2);
  act_pose.orient(0)+=matX(3);
  act_pose.orient(1)+=matX(4);
  act_pose.orient(2)+=matX(5);
  
  //检查是否收敛
  double deltaR = sqrt(pow(matX(3)/Deg2Rad, 2) + pow(matX(4)/Deg2Rad, 2) + pow(matX(5)/Deg2Rad, 2));
  double deltaT = sqrt(pow(matX(0), 2) + pow(matX(1), 2) + pow(matX(2), 2));

  if (deltaR < 0.1 && deltaT < 0.03) {
    return true; // converged
  }
  return false; // keep optimizing
}


pose_type scan2map3d::location(pose_type init_pose, pcl::PointCloud<PointType>::Ptr lidar_cloud){    
  pose_type res_pose = init_pose;
  
  iter_count_=0;

  auto start = std::chrono::system_clock::now();
  while(iter_count_ < max_iteration_){
    surfOptimization(res_pose, lidar_cloud);
    //auto end_seq1 = std::chrono::system_clock::now();
    fitness_ = static_cast<float>(features_.pickup_num) / static_cast<float>(lidar_cloud->points.size());         
    if (LM_optimization(iter_count_, res_pose) == true){
      if(iter_count_>1) break;
    }
    iter_count_++;
  }
  auto end = std::chrono::system_clock::now();
  during_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  return res_pose;
}

